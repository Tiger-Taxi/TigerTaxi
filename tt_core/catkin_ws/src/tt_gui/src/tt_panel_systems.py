from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from PyQt5.QtGui import *

from argparse import ArgumentParser

from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

import numpy as np
import rospkg
import rospy
import math
import time
import os

from stylesheet import *

class tt_panel_systems_ui(QWidget):
    def __init__(self, parent):
        super(tt_panel_systems_ui, self).__init__()
        self.setParent(parent)
        self.setObjectName('tt_panel_systems_ui')
        ui_file = rospkg.RosPack().get_path('tt_gui') + '/resource/' + 'tt_panel_systems_ui.ui'
        loadUi(ui_file, self)
        self.initIMU()
        self.initGPS()
        self.initCam()
        self.initVel()
        self.initHoy()

        self.layout = QVBoxLayout()
        self.layout.addStretch()
        self.layout.addWidget(self.imu_frame)
        self.layout.addStretch()
        self.layout.addWidget(self.gps_frame)
        self.layout.addStretch()
        self.setLayout(self.layout)

    def initIMU(self):
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

        # Degraded | Running | Disabled
        self.imu_status = 0
        # Paused | Resumed
        self.imu_toggle = 1
        # Enabled | Disabled
        self.imu_control = 0
        self.imu_times = np.zeros(HZ_CALC_SIZE)

        self.imu_update()

        if TRACK_SYSTEM_STATUS:
            self.subscriber_IMU = rospy.Subscriber('vectornav/IMU', Imu, self.imu_callback, queue_size=100)

    def initGPS(self):
        self.gps_frame.setStyleSheet(FRAME_STYLE)
        self.gps_label.setStyleSheet(SUB_L_STYLE)

        self.gps_status_layout = QHBoxLayout()
        self.gps_status_layout.addWidget(self.gps_status_label)
        self.gps_status_layout.addWidget(self.gps_status_value)
        self.gps_status_widget.setLayout(self.gps_status_layout)

        self.gps_rate_layout = QHBoxLayout()
        self.gps_rate_layout.addWidget(self.gps_rate_label)
        self.gps_rate_layout.addWidget(self.gps_rate_value)
        self.gps_rate_widget.setLayout(self.gps_rate_layout)

        self.gps_message_label.setStyleSheet(MES_L_STYLE)
        self.gps_message_layout = QVBoxLayout()

        self.gps_lon_layout = QHBoxLayout()
        self.gps_lon_layout.addWidget(self.gps_lon_label)
        self.gps_lon_layout.addWidget(self.gps_lon_value)
        self.gps_lon_widget.setLayout(self.gps_lon_layout)

        self.gps_lat_layout = QHBoxLayout()
        self.gps_lat_layout.addWidget(self.gps_lat_label)
        self.gps_lat_layout.addWidget(self.gps_lat_value)
        self.gps_lat_widget.setLayout(self.gps_lat_layout)

        self.gps_message_layout.addWidget(self.gps_message_label)
        self.gps_message_layout.addWidget(self.gps_lon_widget)
        self.gps_message_layout.addWidget(self.gps_lat_widget)
        self.gps_message_layout.addWidget(self.gps_message_button)
        self.gps_message_widget.setLayout(self.gps_message_layout)

        self.gps_frame_layout = QVBoxLayout()
        self.gps_frame_layout.addWidget(self.gps_label)
        self.gps_frame_layout.addWidget(self.gps_status_widget)
        self.gps_frame_layout.addWidget(self.gps_rate_widget)
        self.gps_frame_layout.addWidget(self.gps_message_widget)
        self.gps_frame_layout.addWidget(self.gps_control_button)
        self.gps_frame.setLayout(self.gps_frame_layout)

        self.gps_status_label.setStyleSheet(MES_L_STYLE)
        self.gps_status_value.setStyleSheet(MES_L_STYLE)
        self.gps_rate_label.setStyleSheet(MES_L_STYLE)
        self.gps_rate_value.setStyleSheet(MES_L_STYLE)
        self.gps_message_label.setStyleSheet(MES_L_STYLE)

        self.gps_lon_label.setStyleSheet(MES_L_STYLE)
        self.gps_lon_value.setStyleSheet(MES_L_STYLE)
        self.gps_lat_label.setStyleSheet(MES_L_STYLE)
        self.gps_lat_value.setStyleSheet(MES_L_STYLE)

        self.gps_message_button.setStyleSheet(SMALL_BUTTON_NORMAL)
        self.gps_control_button.setStyleSheet(SMALL_BUTTON_NORMAL)

        self.gps_message_button.clicked.connect(self.gps_toggle_pause)
        self.gps_control_button.clicked.connect(self.gps_control)

        # Degraded | Running | Disabled
        self.gps_status = 0
        # Paused | Resumed
        self.gps_toggle = 1
        # Enabled | Disabled
        self.gps_control = 0
        self.gps_times = np.zeros(HZ_CALC_SIZE)

        self.gps_update()

        if TRACK_SYSTEM_STATUS:
            self.subscriber_GPS = rospy.Subscriber('vectornav/GPS', NavSatFix, self.gps_callback, queue_size=100)

    def initCam(self):
        pass

    def initVel(self):
        pass

    def initHoy(self):
        pass

    def calcFrequency(self, times):
        t = time.time()
        temp = times
        times = np.roll(times, -1)
        times[-1] = t
        avg_dif = np.mean(times - temp)
        hz = 1 / avg_dif
        return (hz < FREQ_CUTOFF), times, hz

    def imu_update(self):
        if self.imu_status == 0:
            self.imu_status_value.setText(STATUS_LABELS[0])
            self.imu_status_value.setStyleSheet(S_DEGRADED)
        elif self.imu_status == 1:
            self.imu_status_value.setText(STATUS_LABELS[1])
            self.imu_status_value.setStyleSheet(S_RUNNING)
        else:
            self.imu_status_value.setText(STATUS_LABELS[2])
            self.imu_status_value.setStyleSheet(S_DISABLED)

        if self.imu_toggle == 0:
            self.imu_message_button.setText(TOGGLE_LABELS[0])
        else:
            self.imu_message_button.setText(TOGGLE_LABELS[1])

        if self.imu_control == 0:
            self.imu_control_button.setText(CONTROL_LABELS[0])
        else:
            self.imu_control_button.setText(CONTROL_LABELS[1])

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

    # TODO - getting logs that say a filesheet couldn't be parsed - I don't like it
    def imu_callback(self, data):
#        deg, self.imu_times, hz = self.calcFrequency(self.imu_times)
#        if deg != self.imu_status:
#            self.imu_update()
#        self.imu_status = not deg
#        self.imu_rate_value.setText(str(hz))

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

    def gps_update(self):
        if self.gps_status == 0:
            self.gps_status_value.setText(STATUS_LABELS[0])
            self.gps_status_value.setStyleSheet(S_DEGRADED)
        elif self.gps_status == 1:
            self.gps_status_value.setText(STATUS_LABELS[1])
            self.gps_status_value.setStyleSheet(S_RUNNING)
        else:
            self.gps_status_value.setText(STATUS_LABELS[2])
            self.gps_status_value.setStyleSheet(S_DISABLED)

        if self.gps_toggle == 0:
            self.gps_message_button.setText(TOGGLE_LABELS[0])
        else:
            self.gps_message_button.setText(TOGGLE_LABELS[1])

        if self.gps_control == 0:
            self.gps_control_button.setText(CONTROL_LABELS[0])
        else:
            self.gps_control_button.setText(CONTROL_LABELS[1])

    def gps_toggle_pause(self):
        self.gps_toggle = not self.gps_toggle
        self.gps_update()

    def gps_control(self):
        self.gps_control = not self.gps_control
        self.gps_update()
        if self.gps_control == 0:
            self.gps_enable()
        else:
            self.gps_disable()

    def gps_enable(self):
        # TODO - not sure what to do yet
        pass

    def gps_disable(self):
        # TODO - not sure what to do yet
        pass

    # TODO - getting logs that say a filesheet couldn't be parsed - I don't like it
    def gps_callback(self, data):
 #       deg, self.gps_times, hz = self.calcFrequency(self.gps_times)
 #       if deg != self.gps_status:
 #           self.gps_update()
 #       self.gps_status = not deg
 #       self.gps_rate_value.setText(str(hz))

        if self.gps_toggle == 1:
            lon = data.longitude
            lat = data.latitude

            self.gps_lon_value.setText('%2.10f' % lon)
            self.gps_lat_value.setText('%2.10f' % lat)
        else:
            pass