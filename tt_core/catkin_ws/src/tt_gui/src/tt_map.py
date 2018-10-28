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

class tt_map_ui(QWidget):
    def __init__(self):
        super(tt_map_ui, self).__init__()
        self.setObjectName('tt_map_ui')
        ui_file = rospkg.RosPack().get_path('tt_gui') + '/resource/' + 'tt_map_ui.ui'
        loadUi(ui_file, self)

        self.message = 'The test worked!'
        self.button_test.setStyleSheet(BUTTON_NORMAL)
        self.button_test.clicked.connect(self.button_test_pressed)
        self.publisher_greetings = rospy.Publisher('gui_test', String, queue_size = 1)

    def button_test_pressed(self):
        self._publish_mode(None)

    def _publish_mode(self, event):
        self.publisher_greetings.publish(String(self.message))
