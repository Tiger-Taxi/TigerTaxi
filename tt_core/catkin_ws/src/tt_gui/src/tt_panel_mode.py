from python_qt_binding.QtWidgets import QWidget, QPushButton
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

# Class definition for the overarching GUI module
class tt_panel_mode_ui(QWidget):
    # Initiliazation sequence for all GUI components, their respective connections and publisher/subscriber relationships
    def __init__(self, parent):
        super(tt_panel_mode_ui, self).__init__()
        self.setParent(parent)
        self.setObjectName('tt_panel_mode_ui')
        # Get path to UI file which should be in the "resource" folder of this package
        # TODO - hardcoded tt references
        ui_file = rospkg.RosPack().get_path('tt_gui') + '/resource/' + 'tt_panel_mode_ui.ui'
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)

        # Initialize internal mode [0 - hi | 1 - bye]
        self.message = 'Mode: On ...'

        # Initialize buttons with the styles defined above the class definition
        self.button_test.setStyleSheet(style_normal)
        #self.button_test = QPushButton()

        # Connect buttons to functions
        self.button_test.clicked.connect(self.button_test_pressed)

        # Initialize publisher with topic, message type
        self.publisher_greetings = rospy.Publisher('gui_test', String, queue_size = 1)

    # Function called when the "hi" button is pressed
    def button_test_pressed(self):
        self._publish_mode(None)

    def _publish_mode(self, event):
        self.publisher_greetings.publish(String(self.message))
