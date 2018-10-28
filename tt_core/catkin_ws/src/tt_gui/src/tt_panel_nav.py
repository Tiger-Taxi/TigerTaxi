from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from PyQt5.QtGui import *

from argparse import ArgumentParser

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

import rospkg
import rospy
import math
import os

from stylesheet import *

class tt_panel_nav_ui(QWidget):
    def __init__(self, parent):
        super(tt_panel_nav_ui, self).__init__()
        self.setParent(parent)
        self.setObjectName('tt_panel_nav_ui')
        ui_file = rospkg.RosPack().get_path('tt_gui') + '/resource/' + 'tt_panel_nav_ui.ui'
        loadUi(ui_file, self)

        self.cur_pos_frame.setStyleSheet(FRAME_STYLE)
        self.cur_pos_label.setStyleSheet(LABEL_STYLE)

        self.cur_long_layout = QHBoxLayout()
        self.cur_long_layout.addWidget(self.label_cur_long)
        self.cur_long_layout.addWidget(self.value_cur_long)
        self.cur_long_widget.setLayout(self.cur_long_layout)

        self.cur_lat_layout = QHBoxLayout()
        self.cur_lat_layout.addWidget(self.label_cur_lat)
        self.cur_lat_layout.addWidget(self.value_cur_lat)
        self.cur_lat_widget.setLayout(self.cur_lat_layout)

        self.cur_yaw_layout = QHBoxLayout()
        self.cur_yaw_layout.addWidget(self.label_cur_yaw)
        self.cur_yaw_layout.addWidget(self.value_cur_yaw)
        self.cur_yaw_widget.setLayout(self.cur_yaw_layout)

        self.cur_pos_layout = QVBoxLayout()
        self.cur_pos_layout.addWidget(self.cur_pos_label)
        self.cur_pos_layout.addWidget(self.cur_long_widget)
        self.cur_pos_layout.addWidget(self.cur_lat_widget)
        self.cur_pos_layout.addWidget(self.cur_yaw_widget)
        self.cur_pos_frame.setLayout(self.cur_pos_layout)

        self.goal_pos_frame.setStyleSheet(FRAME_STYLE)
        self.goal_pos_label.setStyleSheet(LABEL_STYLE)

        self.goal_status_layout = QHBoxLayout()
        self.goal_status_layout.addWidget(self.label_goal_status)
        self.goal_status_layout.addWidget(self.value_goal_status)
        self.goal_status_widget.setLayout(self.goal_status_layout)

        self.goal_long_layout = QHBoxLayout()
        self.goal_long_layout.addWidget(self.label_goal_long)
        self.goal_long_layout.addWidget(self.value_goal_long)
        self.goal_long_widget.setLayout(self.goal_long_layout)

        self.goal_lat_layout = QHBoxLayout()
        self.goal_lat_layout.addWidget(self.label_goal_lat)
        self.goal_lat_layout.addWidget(self.value_goal_lat)
        self.goal_lat_widget.setLayout(self.goal_lat_layout)

        self.goal_yaw_layout = QHBoxLayout()
        self.goal_yaw_layout.addWidget(self.label_goal_yaw)
        self.goal_yaw_layout.addWidget(self.value_goal_yaw)
        self.goal_yaw_widget.setLayout(self.goal_yaw_layout)

        self.goal_pos_layout = QVBoxLayout()
        self.goal_pos_layout.addWidget(self.goal_pos_label)
        self.goal_pos_layout.addWidget(self.goal_status_widget)
        self.goal_pos_layout.addWidget(self.goal_long_widget)
        self.goal_pos_layout.addWidget(self.goal_lat_widget)
        self.goal_pos_layout.addWidget(self.goal_yaw_widget)
        self.goal_pos_frame.setLayout(self.goal_pos_layout)

        self.button_follow.clicked.connect(self.button_follow_pressed)
        self.button_pause.clicked.connect(self.button_pause_pressed)
        self.button_cancel.clicked.connect(self.button_cancel_pressed)

        self.nav_control_layout = QHBoxLayout()
        self.nav_control_layout.addWidget(self.button_follow)
        self.nav_control_layout.addWidget(self.button_pause)
        self.nav_control_layout.addWidget(self.button_cancel)
        self.nav_control_widget.setLayout(self.nav_control_layout)

        self.layout = QVBoxLayout()
        self.layout.addStretch()
        self.layout.addWidget(self.cur_pos_frame)
        self.layout.addStretch()
        self.layout.addWidget(self.goal_pos_frame)
        self.layout.addStretch()
        self.layout.addWidget(self.nav_control_widget)
        self.layout.addStretch()
        self.setLayout(self.layout)

        self.label_cur_long.setStyleSheet(SUB_L_STYLE)
        self.value_cur_long.setStyleSheet(SUB_L_STYLE)
        self.label_cur_lat.setStyleSheet(SUB_L_STYLE)
        self.value_cur_lat.setStyleSheet(SUB_L_STYLE)
        self.label_cur_yaw.setStyleSheet(SUB_L_STYLE)
        self.value_cur_yaw.setStyleSheet(SUB_L_STYLE)

        self.label_goal_status.setStyleSheet(SUB_L_STYLE)
        self.value_goal_status.setStyleSheet(SUB_L_STYLE)
        self.label_goal_long.setStyleSheet(SUB_L_STYLE)
        self.value_goal_long.setStyleSheet(SUB_L_STYLE)
        self.label_goal_lat.setStyleSheet(SUB_L_STYLE)
        self.value_goal_lat.setStyleSheet(SUB_L_STYLE)
        self.label_goal_yaw.setStyleSheet(SUB_L_STYLE)
        self.value_goal_yaw.setStyleSheet(SUB_L_STYLE)

        # Not Set | Confirm | Following
        self.navstatus = 0
        self._update_button_state()

        self.subscriber_GPS = rospy.Subscriber('vectornav/GPS', NavSatFix, self.gps_callback, queue_size=10)
        self.subscriber_IMU = rospy.Subscriber('vectornav/IMU', Imu, self.imu_callback, queue_size=10)

    def button_follow_pressed(self):
        # TODO - Need to add in all of the logic here
        self._update_button_state()

    def button_pause_pressed(self):
        # TODO - Need to add in all of the logic here
        self._update_button_state()

    def button_cancel_pressed(self):
        # TODO - Need to add in all of the logic here
        self._update_button_state()

    def _update_button_state(self):
        if self.navstatus == 0:
            # TODO - more logic needs to be added here
            styles = [BUTTON_OFF, BUTTON_OFF, BUTTON_OFF]
        elif self.navstatus == 1:
            # TODO - more logic needs to be added here
            styles = [BUTTON_NORMAL, BUTTON_OFF, BUTTON_OFF]
        else:
            # TODO - more logic needs to be added here
            styles = [BUTTON_OFF, BUTTON_NORMAL, BUTTON_NORMAL]

        self.button_follow.setStyleSheet(styles[0])
        self.button_pause.setStyleSheet(styles[1])
        self.button_cancel.setStyleSheet(styles[2])

    def gps_callback(self, data):
        lon = data.longitude
        lat = data.latitude

        self.value_cur_long.setText('%2.10f' % lon)
        self.value_cur_lat.setText('%2.10f' % lat)

    def imu_callback(self, data):
        orien = data.orientation
        x = orien.x
        y = orien.y
        z = orien.z
        w = orien.w

        # Quaternion to yaw conversion
        siny_cosp = 2.0 * (w * z + x * y);
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z);  
        yaw = math.atan2(siny_cosp, cosy_cosp);

        self.value_cur_yaw.setText('%3.5f' % math.degrees(yaw))
