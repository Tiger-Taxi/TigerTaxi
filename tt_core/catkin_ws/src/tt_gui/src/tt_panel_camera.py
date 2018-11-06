from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from PyQt5.QtGui import *

from argparse import ArgumentParser

from std_msgs.msg import Float32
from std_msgs.msg import String

import rospkg
import rospy
import os

from stylesheet import *

# Widget for the camera panel (NOT IMPLEMENTED)
class tt_panel_camera_ui(QWidget):
    def __init__(self, parent):
        super(tt_panel_camera_ui, self).__init__()
        self.setParent(parent)
        self.setObjectName('tt_panel_camera_ui')
        ui_file = rospkg.RosPack().get_path('tt_gui') + '/resource/' + 'tt_panel_camera_ui.ui'
        loadUi(ui_file, self)

        # Add the 'not implemented' label
        self.not_impl.setStyleSheet(SUB_L_STYLE)
