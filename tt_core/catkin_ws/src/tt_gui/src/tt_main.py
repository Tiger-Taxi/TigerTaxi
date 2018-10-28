from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from PyQt5.QtGui import *

from argparse import ArgumentParser

import rospkg
import rospy
import os

from tt_map import tt_map_ui
from tt_panels import tt_panels_ui

from stylesheet import *

class tt_main_ui(Plugin):
    def __init__(self, context):
        super(tt_main_ui, self).__init__(context)
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action = "store_true", dest = "quiet", help = "Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            rospy.loginfo('tt_gui->tt_main arguments: ' + str(args))
            rospy.loginfo('tt_gui->tt_main unknowns: ' + str(unknowns))

        self.main_widget = QWidget()
        ui_file = rospkg.RosPack().get_path('tt_gui') + '/resource/' + 'tt_main_ui.ui'
        loadUi(ui_file, self.main_widget)
        context.add_widget(self.main_widget)

        self.map_widget = tt_map_ui()
        self.panels_widget = tt_panels_ui()

        self.layout = QHBoxLayout()
        self.layout.addWidget(self.map_widget, MAP_WEIGHT)
        self.layout.addWidget(self.panels_widget, PAN_WEIGHT)
        self.main_widget.setLayout(self.layout)
