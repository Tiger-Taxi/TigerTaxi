from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from PyQt5.QtGui import *

from argparse import ArgumentParser

from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

import gui_freq_listener
import numpy as np
import rostopic
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

        self.hz = rostopic.ROSTopicHz(-1)

        self.initIMU()
        self.initGPS()
        self.initCam()
        self.initVel()
        self.initHok()

        self.layout = QVBoxLayout()
        self.layout.addStretch()
        self.layout.addWidget(self.imu_frame)
        self.layout.addStretch()
        self.layout.addWidget(self.gps_frame)
        self.layout.addStretch()
        self.layout.addWidget(self.cam_frame)
        self.layout.addStretch()
        self.layout.addWidget(self.vel_frame)
        self.layout.addStretch()
        self.layout.addWidget(self.hok_frame)
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

        self.imu_update()

        if TRACK_SYSTEM_STATUS:
            self.subscriber_IMU = rospy.Subscriber('tt_gui/throttle/imu', Imu, self.imu_callback, queue_size = QUEUE_SIZE)
            self.imu_hz = rospy.Subscriber('/vectornav/IMU', rospy.AnyMsg, self.hz.callback_hz, callback_args='/vectornav/IMU')

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

        self.gps_update()

        if TRACK_SYSTEM_STATUS:
            self.subscriber_GPS = rospy.Subscriber('tt_gui/throttle/gps', NavSatFix, self.gps_callback, queue_size=QUEUE_SIZE)
            self.gps_hz = rospy.Subscriber('/vectornav/GPS', rospy.AnyMsg, self.hz.callback_hz, callback_args='/vectornav/GPS')

    def initCam(self):
        self.cam_frame.setStyleSheet(FRAME_STYLE)
        self.cam_label.setStyleSheet(SUB_L_STYLE)

        self.cam_status_layout = QHBoxLayout()
        self.cam_status_layout.addWidget(self.cam_status_label)
        self.cam_status_layout.addWidget(self.cam_status_value)
        self.cam_status_widget.setLayout(self.cam_status_layout)

        self.cam_rate_layout = QHBoxLayout()
        self.cam_rate_layout.addWidget(self.cam_rate_label)
        self.cam_rate_layout.addWidget(self.cam_rate_value)
        self.cam_rate_widget.setLayout(self.cam_rate_layout)

        self.cam_frame_layout = QVBoxLayout()
        self.cam_frame_layout.addWidget(self.cam_label)
        self.cam_frame_layout.addWidget(self.cam_status_widget)
        self.cam_frame_layout.addWidget(self.cam_rate_widget)
        self.cam_frame_layout.addWidget(self.cam_control_button)
        self.cam_frame.setLayout(self.cam_frame_layout)

        self.cam_status_label.setStyleSheet(MES_L_STYLE)
        self.cam_status_value.setStyleSheet(MES_L_STYLE)
        self.cam_rate_label.setStyleSheet(MES_L_STYLE)
        self.cam_rate_value.setStyleSheet(MES_L_STYLE)

        self.cam_control_button.setStyleSheet(SMALL_BUTTON_NORMAL)

        self.cam_control_button.clicked.connect(self.cam_control)

        # Degraded | Running | Disabled
        self.cam_status = 0
        # Enabled | Disabled
        self.cam_control = 0

        self.cam_update()

    def initVel(self):
        self.vel_frame.setStyleSheet(FRAME_STYLE)
        self.vel_label.setStyleSheet(SUB_L_STYLE)

        self.vel_status_layout = QHBoxLayout()
        self.vel_status_layout.addWidget(self.vel_status_label)
        self.vel_status_layout.addWidget(self.vel_status_value)
        self.vel_status_widget.setLayout(self.vel_status_layout)

        self.vel_rate_layout = QHBoxLayout()
        self.vel_rate_layout.addWidget(self.vel_rate_label)
        self.vel_rate_layout.addWidget(self.vel_rate_value)
        self.vel_rate_widget.setLayout(self.vel_rate_layout)

        self.vel_frame_layout = QVBoxLayout()
        self.vel_frame_layout.addWidget(self.vel_label)
        self.vel_frame_layout.addWidget(self.vel_status_widget)
        self.vel_frame_layout.addWidget(self.vel_rate_widget)
        self.vel_frame_layout.addWidget(self.vel_control_button)
        self.vel_frame.setLayout(self.vel_frame_layout)

        self.vel_status_label.setStyleSheet(MES_L_STYLE)
        self.vel_status_value.setStyleSheet(MES_L_STYLE)
        self.vel_rate_label.setStyleSheet(MES_L_STYLE)
        self.vel_rate_value.setStyleSheet(MES_L_STYLE)

        self.vel_control_button.setStyleSheet(SMALL_BUTTON_NORMAL)

        self.vel_control_button.clicked.connect(self.vel_control)

        # Degraded | Running | Disabled
        self.vel_status = 0
        # Enabled | Disabled
        self.vel_control = 0

        self.vel_update()

    def initHok(self):
        self.hok_frame.setStyleSheet(FRAME_STYLE)
        self.hok_label.setStyleSheet(SUB_L_STYLE)

        self.hok_status_layout = QHBoxLayout()
        self.hok_status_layout.addWidget(self.hok_status_label)
        self.hok_status_layout.addWidget(self.hok_status_value)
        self.hok_status_widget.setLayout(self.hok_status_layout)

        self.hok_rate_layout = QHBoxLayout()
        self.hok_rate_layout.addWidget(self.hok_rate_label)
        self.hok_rate_layout.addWidget(self.hok_rate_value)
        self.hok_rate_widget.setLayout(self.hok_rate_layout)

        self.hok_frame_layout = QVBoxLayout()
        self.hok_frame_layout.addWidget(self.hok_label)
        self.hok_frame_layout.addWidget(self.hok_status_widget)
        self.hok_frame_layout.addWidget(self.hok_rate_widget)
        self.hok_frame_layout.addWidget(self.hok_control_button)
        self.hok_frame.setLayout(self.hok_frame_layout)

        self.hok_status_label.setStyleSheet(MES_L_STYLE)
        self.hok_status_value.setStyleSheet(MES_L_STYLE)
        self.hok_rate_label.setStyleSheet(MES_L_STYLE)
        self.hok_rate_value.setStyleSheet(MES_L_STYLE)

        self.hok_control_button.setStyleSheet(SMALL_BUTTON_NORMAL)

        self.hok_control_button.clicked.connect(self.hok_control)

        # Degraded | Running | Disabled
        self.hok_status = 0
        # Enabled | Disabled
        self.hok_control = 0

        self.hok_update()

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
            rate = self.hz.get_hz('/vectornav/IMU')
            if rate is not None:
                self.imu_rate_value.setText('%3.5f' % rate[0])
                if rate[0] > FREQ_CUTOFF:
                    self.imu_status = 1
                else:
                    self.imu_status = 0
                self.imu_update()
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
        if self.gps_toggle == 1:
            lon = data.longitude
            lat = data.latitude

            self.gps_lon_value.setText('%2.10f' % lon)
            self.gps_lat_value.setText('%2.10f' % lat)
            rate = self.hz.get_hz('/vectornav/GPS')
            if rate is not None:
                self.gps_rate_value.setText('%3.5f' % rate[0])
                if rate[0] > FREQ_CUTOFF:
                    self.gps_status = 1
                else:
                    self.gps_status = 0
                self.gps_update()
        else:
            pass

    def cam_update(self):
        if self.cam_status == 0:
            self.cam_status_value.setText(STATUS_LABELS[0])
            self.cam_status_value.setStyleSheet(S_DEGRADED)
        elif self.cam_status == 1:
            self.cam_status_value.setText(STATUS_LABELS[1])
            self.cam_status_value.setStyleSheet(S_RUNNING)
        else:
            self.cam_status_value.setText(STATUS_LABELS[2])
            self.cam_status_value.setStyleSheet(S_DISABLED)

        if self.cam_control == 0:
            self.cam_control_button.setText(CONTROL_LABELS[0])
        else:
            self.cam_control_button.setText(CONTROL_LABELS[1])

    def cam_control(self):
        self.cam_control = not self.cam_control
        self.cam_update()
        if self.cam_control == 0:
            self.cam_enable()
        else:
            self.cam_disable()

    def cam_enable(self):
        # TODO - not sure what to do yet
        pass

    def cam_disable(self):
        # TODO - not sure what to do yet
        pass

    def vel_update(self):
        if self.vel_status == 0:
            self.vel_status_value.setText(STATUS_LABELS[0])
            self.vel_status_value.setStyleSheet(S_DEGRADED)
        elif self.vel_status == 1:
            self.vel_status_value.setText(STATUS_LABELS[1])
            self.vel_status_value.setStyleSheet(S_RUNNING)
        else:
            self.vel_status_value.setText(STATUS_LABELS[2])
            self.vel_status_value.setStyleSheet(S_DISABLED)

        if self.vel_control == 0:
            self.vel_control_button.setText(CONTROL_LABELS[0])
        else:
            self.vel_control_button.setText(CONTROL_LABELS[1])

    def vel_control(self):
        self.vel_control = not self.vel_control
        self.vel_update()
        if self.vel_control == 0:
            self.vel_enable()
        else:
            self.vel_disable()

    def vel_enable(self):
        # TODO - not sure what to do yet
        pass

    def vel_disable(self):
        # TODO - not sure what to do yet
        pass

    def hok_update(self):
        if self.hok_status == 0:
            self.hok_status_value.setText(STATUS_LABELS[0])
            self.hok_status_value.setStyleSheet(S_DEGRADED)
        elif self.hok_status == 1:
            self.hok_status_value.setText(STATUS_LABELS[1])
            self.hok_status_value.setStyleSheet(S_RUNNING)
        else:
            self.hok_status_value.setText(STATUS_LABELS[2])
            self.hok_status_value.setStyleSheet(S_DISABLED)

        if self.hok_control == 0:
            self.hok_control_button.setText(CONTROL_LABELS[0])
        else:
            self.hok_control_button.setText(CONTROL_LABELS[1])

    def hok_control(self):
        self.hok_control = not self.hok_control
        self.hok_update()
        if self.hok_control == 0:
            self.hok_enable()
        else:
            self.hok_disable()

    def hok_enable(self):
        # TODO - not sure what to do yet
        pass

    def hok_disable(self):
        # TODO - not sure what to do yet
        pass
