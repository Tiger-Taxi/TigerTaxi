from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from PyQt5.QtGui import *

from argparse import ArgumentParser

from std_msgs.msg import Float32
from std_msgs.msg import String

import rospkg
import rospy
import os

# Define styles for buttons
style_normal = """color: black;font-size: 26px;"""
style_active = """color: green;font-size: 26px;"""

# Class definition for the overarching GUI module
class tt_main_ui(Plugin):
    # Initiliazation sequence for all GUI components, their respective connections and publisher/subscriber relationships
    def __init__(self, context):
        super(tt_main_ui, self).__init__(context)
        # Process standalone plugin command-line arguments
        parser = ArgumentParser()
        # TODO - what do these do and what else can be added here
        # Add relevant arguments to be parsed
        parser.add_argument("-q", "--quiet", action = "store_true", dest = "quiet", help = "Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            # TODO - hardcoded tt references
            rospy.loginfo('tt_gui->tt_main arguments: ' + str(args))
            rospy.loginfo('tt_gui->tt_main unknowns: ' + str(unknowns))

        # Create QWidget for the overarching GUI module
        self.main_widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        # TODO - hardcoded tt references
        ui_file = rospkg.RosPack().get_path('tt_gui') + '/resource/' + 'tt_main_ui.ui'
        rospy.loginfo('UI file %s specified for display' % ui_file)
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self.main_widget)
        # Add widget to the user interface
        context.add_widget(self.main_widget)

        # Initialize internal mode [0 - hi | 1 - bye]
        self.mode = 1

        # Initialize buttons with the styles defined above the class definition
       	self.main_widget.button_hi.setStyleSheet(style_normal)
        self.main_widget.button_bye.setStyleSheet(style_active)

        # Connect buttons to functions
        self.main_widget.button_hi.clicked.connect(self.button_hi_pressed)
        self.main_widget.button_bye.clicked.connect(self.button_bye_pressed)

        # Call the update function of the button for state updates
        self._update_button_state()

        # Initialize publisher with topic, message type
        self.publisher_greetings = rospy.Publisher('tt_greetings', Float32, queue_size = 1)

    # Function called when the "hi" button is pressed
    def button_hi_pressed(self):
        self.mode = 0
        self._update_button_state()
        self._publish_mode(None)

    # Function called when the "bye" button is pressed
    def button_bye_pressed(self):
        self.mode = 1
        self._update_button_state()
        self._publish_mode(None)

    # Updates the state of the "hi" and "bye" buttons to reflect the internal state
    def _update_button_state(self):
        if self.mode == 0:
            self.main_widget.button_hi.setStyleSheet(style_normal)
            self.main_widget.button_bye.setStyleSheet(style_active)
        else:
            self.main_widget.button_hi.setStyleSheet(style_active)
            self.main_widget.button_bye.setStyleSheet(style_normal)

    def _publish_mode(self, event):
        self.publisher_greetings.publish(Float32(self.mode))
