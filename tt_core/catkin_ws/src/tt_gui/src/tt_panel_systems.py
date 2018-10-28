from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from PyQt5.QtGui import *

from argparse import ArgumentParser

from sensor_msgs.msg import Imu

import numpy as np
import rospkg
import rospy
import math
import time
import os

from stylesheet import *

# Class definition for the overarching GUI module
class tt_panel_systems_ui(QWidget):
    # Initiliazation sequence for all GUI components, their respective connections and publisher/subscriber relationships
    def __init__(self, parent):
        super(tt_panel_systems_ui, self).__init__()
        self.setParent(parent)
        self.setObjectName('tt_panel_systems_ui')
        # Get path to UI file which should be in the "resource" folder of this package
        # TODO - hardcoded tt references
        ui_file = rospkg.RosPack().get_path('tt_gui') + '/resource/' + 'tt_panel_systems_ui.ui'
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)

        self.imu_frame.setStyleSheet(FRAME_STYLE)
        self.imu_label.setStyleSheet(SUB_L_STYLE)

        self.imu_status_layout = QHBoxLayout()
        self.imu_status_layout.addWidget(self.imu_status_label)
        self.imu_status_layout.addWidget(self.imu_status_value)
        self.imu_status_widget.setLayout(self.imu_status_layout)

        self.imu_rate_layout = QHBoxLayout()
        self.imu_rate_layout.addWidget(self.imu_rate_label)
        self.imu_rate_layout.addWidget(self.imu_rate_value)
        self.imu_rate_widget.setLayout(self.imu_rate_layout)

        self.imu_message_label.setStyleSheet(MES_L_STYLE)
        self.imu_message_layout = QVBoxLayout()

        self.imu_yaw_layout = QHBoxLayout()
        self.imu_yaw_layout.addWidget(self.imu_yaw_label)
        self.imu_yaw_layout.addWidget(self.imu_yaw_value)
        self.imu_yaw_widget.setLayout(self.imu_yaw_layout)

        self.imu_message_layout.addWidget(self.imu_message_label)
        self.imu_message_layout.addWidget(self.imu_yaw_widget)
        self.imu_message_layout.addWidget(self.imu_message_button)
        self.imu_message_widget.setLayout(self.imu_message_layout)

        self.imu_frame_layout = QVBoxLayout()
        self.imu_frame_layout.addWidget(self.imu_label)
        self.imu_frame_layout.addWidget(self.imu_status_widget)
        self.imu_frame_layout.addWidget(self.imu_rate_widget)
        self.imu_frame_layout.addWidget(self.imu_message_widget)
        self.imu_frame_layout.addWidget(self.imu_control_button)
        self.imu_frame.setLayout(self.imu_frame_layout)

        self.layout = QVBoxLayout()
        self.layout.addStretch()
        self.layout.addWidget(self.imu_frame)
        self.layout.addStretch()
        self.setLayout(self.layout)

        self.imu_status_label.setStyleSheet(MES_L_STYLE)
        self.imu_status_value.setStyleSheet(MES_L_STYLE)
        self.imu_rate_label.setStyleSheet(MES_L_STYLE)
        self.imu_rate_value.setStyleSheet(MES_L_STYLE)
        self.imu_message_label.setStyleSheet(MES_L_STYLE)

        self.imu_yaw_label.setStyleSheet(MES_L_STYLE)
        self.imu_yaw_value.setStyleSheet(MES_L_STYLE)

        self.imu_message_button.setStyleSheet(SMALL_BUTTON_NORMAL)
        self.imu_control_button.setStyleSheet(SMALL_BUTTON_NORMAL)

        self.imu_message_button.clicked.connect(self.imu_toggle_pause)
        self.imu_control_button.clicked.connect(self.imu_control)

        self.imu_times = np.zeros(HZ_CALC_SIZE)

        # Degraded | Running | Disabled
        self.imu_status = 0
        # Paused | Resumed
        self.imu_toggle = 1
        # Enabled | Disabled
        self.imu_control = 0

        self.imu_update()

        self.subscriber_IMU = rospy.Subscriber('vectornav/IMU', Imu, self.imu_callback, queue_size=10)

    def imu_update(self):
        status_labels = ['Degraded', 'Running', 'Disabled']
        toggle_labels = ['Live Feed', 'Pause']
        control_labels = ['Disable', 'Enable']

        if self.imu_status == 0:
            self.imu_status_value.setText(status_labels[0])
            self.imu_status_value.setStyleSheet(S_DEGRADED)
        elif self.imu_status == 1:
            self.imu_status_value.setText(status_labels[1])
            self.imu_status_value.setStyleSheet(S_RUNNING)
        else:
            self.imu_status_value.setText(status_labels[2])
            self.imu_status_value.setStyleSheet(S_DISABLED)

        if self.imu_toggle == 0:
            self.imu_message_button.setText(toggle_labels[0])
        else:
            self.imu_message_button.setText(toggle_labels[1])

        if self.imu_control == 0:
            self.imu_control_button.setText(control_labels[0])
        else:
            self.imu_control_button.setText(control_labels[1])

    def imu_toggle_pause(self):
        self.imu_toggle = not self.imu_toggle
        self.imu_update()

    def imu_control(self):
        self.imu_control = not self.imu_control
        self.imu_update()
        if self.imu_control == 0:
            self.imu_enable()
        else:
            self.imu_disable()

    def imu_enable(self):
        # TODO - not sure what to do yet
        pass

    def imu_disable(self):
        # TODO - not sure what to do yet
        pass

    def calcFrequency(self, times):
        t = time.time()
        temp = times
        times = np.roll(times, -1)
        times[-1] = t
        avg_dif = np.mean(times - temp)
        hz = 1 / avg_dif

        return (hz < FREQ_CUTOFF), times, hz

    def imu_callback(self, data):
        deg, self.imu_times, hz = self.calcFrequency(self.imu_times)
        if deg != self.imu_status:
            self.imu_update()
        self.imu_status = not deg
        self.imu_rate_value.setText(str(hz))

        if self.imu_toggle == 1:
            orien = data.orientation
            x = orien.x
            y = orien.y
            z = orien.z
            w = orien.w

            # Quaternion to yaw conversion
            siny_cosp = 2.0 * (w * z + x * y);
            cosy_cosp = 1.0 - 2.0 * (y * y + z * z);  
            yaw = math.atan2(siny_cosp, cosy_cosp);

            self.imu_yaw_value.setText('%3.5f' % math.degrees(yaw))
        else:
            pass
