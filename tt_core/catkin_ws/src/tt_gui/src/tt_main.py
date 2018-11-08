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

# Top-level gui plugin selectable through rqt
class tt_main_ui(Plugin):
    def __init__(self, context):
        super(tt_main_ui, self).__init__(context)

        # Initialize main widget, add it to the context, and load the ui file
        self.main_widget = QWidget()
        ui_file = rospkg.RosPack().get_path('tt_gui') + '/resource/' + 'tt_main_ui.ui'
        loadUi(ui_file, self.main_widget)
        context.add_widget(self.main_widget)

        # Initialize widgets for map and panels
        self.map_widget = tt_map_ui()
        self.panels_widget = tt_panels_ui()

        # Create a layout for the main widget, and add the widgets for map and panels
        self.layout = QHBoxLayout()
        self.layout.addWidget(self.map_widget, MAP_WEIGHT)
        self.layout.addWidget(self.panels_widget, PAN_WEIGHT)
        self.main_widget.setLayout(self.layout)

        # Set the gui size to be that of the available screen geometry
        rect = QDesktopWidget().availableGeometry()
        width, height = rect.width(), rect.height() - rect.y()
        self.main_widget.setFixedSize(width, height)
