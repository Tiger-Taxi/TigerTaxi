from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from PyQt5.QtGui import *

from argparse import ArgumentParser

from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

import numpy as np
import threading
import rostopic
import rospkg
import rospy
import math
import time
import os

from stylesheet import *

# Thread to observe sensor frequencies
class HzThread(object):
    def __init__(self, systems_panel, hz_topic):
        self.systems_panel = systems_panel
        self.hz_topic = hz_topic

        self.thread = threading.Thread(target=self.run, args=())
        self.thread.daemon = True
        self.thread.start()

    def run(self):
        while not rospy.is_shutdown():
            rate = self.hz_topic.get_hz('/vectornav/IMU')
            if rate is not None:
                self.systems_panel.imu_rate_value.setText('%3.5f' % rate[0])
                if rate[0] > FREQ_CUTOFF:
                    self.systems_panel.imu_status = 1
                else:
                    self.systems_panel.imu_status = 0
            else:
                self.systems_panel.imu_rate_value.setText('%3.5f' % 0)
                self.systems_panel.imu_status = 2
            self.systems_panel.imu_update()

            rate = self.hz_topic.get_hz('/vectornav/GPS')
            if rate is not None:
                self.systems_panel.gps_rate_value.setText('%3.5f' % rate[0])
                if rate[0] > FREQ_CUTOFF:
                    self.systems_panel.gps_status = 1
                else:
                    self.systems_panel.gps_status = 0
            else:
                self.systems_panel.gps_rate_value.setText('%3.5f' % 0)
                self.systems_panel.gps_status = 2
            self.systems_panel.gps_update()

            rate = self.hz_topic.get_hz('/enet/image')
            if rate is not None:
                self.systems_panel.cam_rate_value.setText('%3.5f' % rate[0])
                if rate[0] > FREQ_CUTOFF:
                    self.systems_panel.cam_status = 1
                else:
                    self.systems_panel.cam_status = 0
            else:
                self.systems_panel.cam_rate_value.setText('%3.5f' % 0)
                self.systems_panel.cam_status = 2
            self.systems_panel.cam_update()

            rate = self.hz_topic.get_hz('/velodyne_points')
            if rate is not None:
                self.systems_panel.vel_rate_value.setText('%3.5f' % rate[0])
                if rate[0] > FREQ_CUTOFF:
                    self.systems_panel.vel_status = 1
                else:
                    self.systems_panel.vel_status = 0
            else:
                self.systems_panel.vel_rate_value.setText('%3.5f' % 0)
                self.systems_panel.vel_status = 2
            self.systems_panel.vel_update()

            rate = self.hz_topic.get_hz('/scan')
            if rate is not None:
                self.systems_panel.hok_rate_value.setText('%3.5f' % rate[0])
                if rate[0] > FREQ_CUTOFF:
                    self.systems_panel.hok_status = 1
                else:
                    self.systems_panel.hok_status = 0
            else:
                self.systems_panel.hok_rate_value.setText('%3.5f' % 0)
                self.systems_panel.hok_status = 2
            self.systems_panel.hok_update()

            rospy.sleep(FREQ_UPDATE)

# Widget for the subsystems panel
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

        self.hz_thread = HzThread(self, self.hz)

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
        self.imu_frame.setLayout(self.imu_frame_layout)

        self.imu_status_label.setStyleSheet(MES_L_STYLE)
        self.imu_status_value.setStyleSheet(MES_L_STYLE)
        self.imu_rate_label.setStyleSheet(MES_L_STYLE)
        self.imu_rate_value.setStyleSheet(MES_L_STYLE)
        self.imu_message_label.setStyleSheet(MES_L_STYLE)

        self.imu_yaw_label.setStyleSheet(MES_L_STYLE)
        self.imu_yaw_value.setStyleSheet(MES_L_STYLE)

        self.imu_message_button.setStyleSheet(SMALL_BUTTON_NORMAL)

        self.imu_message_button.clicked.connect(self.imu_toggle_pause)

        # Degraded | Running | No Messages
        self.imu_status = 2
        # Paused | Resumed
        self.imu_toggle = 1

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

        self.gps_message_button.clicked.connect(self.gps_toggle_pause)

        # Degraded | Running | No Messages
        self.gps_status = 2
        # Paused | Resumed
        self.gps_toggle = 1

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
        self.cam_frame.setLayout(self.cam_frame_layout)

        self.cam_status_label.setStyleSheet(MES_L_STYLE)
        self.cam_status_value.setStyleSheet(MES_L_STYLE)
        self.cam_rate_label.setStyleSheet(MES_L_STYLE)
        self.cam_rate_value.setStyleSheet(MES_L_STYLE)

        # Degraded | Running | No Messages
        self.cam_status = 2

        self.cam_update()

        if TRACK_SYSTEM_STATUS:
            self.cam_hz = rospy.Subscriber('/enet/image', rospy.AnyMsg, self.hz.callback_hz, callback_args='/enet/image')

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
        self.vel_frame.setLayout(self.vel_frame_layout)

        self.vel_status_label.setStyleSheet(MES_L_STYLE)
        self.vel_status_value.setStyleSheet(MES_L_STYLE)
        self.vel_rate_label.setStyleSheet(MES_L_STYLE)
        self.vel_rate_value.setStyleSheet(MES_L_STYLE)

        # Degraded | Running | No Messages
        self.vel_status = 2

        self.vel_update()

        if TRACK_SYSTEM_STATUS:
            self.vel_hz = rospy.Subscriber('/velodyne_points', rospy.AnyMsg, self.hz.callback_hz, callback_args='/velodyne_points')

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
        self.hok_frame.setLayout(self.hok_frame_layout)

        self.hok_status_label.setStyleSheet(MES_L_STYLE)
        self.hok_status_value.setStyleSheet(MES_L_STYLE)
        self.hok_rate_label.setStyleSheet(MES_L_STYLE)
        self.hok_rate_value.setStyleSheet(MES_L_STYLE)

        # Degraded | Running | No Messages
        self.hok_status = 2

        self.hok_update()

        if TRACK_SYSTEM_STATUS:
            self.hok_hz = rospy.Subscriber('/scan', rospy.AnyMsg, self.hz.callback_hz, callback_args='/scan')

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

    def imu_toggle_pause(self):
        self.imu_toggle = not self.imu_toggle
        self.imu_update()

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

    def gps_toggle_pause(self):
        self.gps_toggle = not self.gps_toggle
        self.gps_update()

    def gps_callback(self, data):
        if self.gps_toggle == 1:
            lon = data.longitude
            lat = data.latitude

            self.gps_lon_value.setText('%2.10f' % lon)
            self.gps_lat_value.setText('%2.10f' % lat)
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
