from python_qt_binding.QtWidgets import QWidget, QHBoxLayout
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from PyQt5.QtGui import *

from argparse import ArgumentParser

from std_msgs.msg import Float32
from std_msgs.msg import String

import rospkg
import rospy
import os

from tt_map import tt_map_ui
from tt_panels import tt_panels_ui

from stylesheet import *

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
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self.main_widget)
        # Add widget to the user interface
        context.add_widget(self.main_widget)

        self.map_widget = tt_map_ui()
        self.panels_widget = tt_panels_ui()

        self.layout = QHBoxLayout()
        self.layout.addWidget(self.map_widget)
        self.layout.addWidget(self.panels_widget)
        self.main_widget.setLayout(self.layout)
