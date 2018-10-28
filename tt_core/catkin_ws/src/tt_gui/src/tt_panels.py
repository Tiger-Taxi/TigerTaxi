from python_qt_binding.QtWidgets import QStackedWidget, QWidget, QHBoxLayout, QVBoxLayout
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from PyQt5.QtGui import *

from argparse import ArgumentParser

from std_msgs.msg import Float32
from std_msgs.msg import String

import rospkg
import rospy
import os

from tt_panel_nav import tt_panel_nav_ui
from tt_panel_mode import tt_panel_mode_ui
from tt_panel_camera import tt_panel_camera_ui
from tt_panel_systems import tt_panel_systems_ui

# Define styles for buttons
style_normal = """color: black;font-size: 26px;"""
style_active = """color: green;font-size: 26px;"""

# Class definition for the overarching GUI module
class tt_panels_ui(QWidget):
    # Initiliazation sequence for all GUI components, their respective connections and publisher/subscriber relationships
    def __init__(self, parent):
        super(tt_panels_ui, self).__init__()
        self.setParent(parent)
        # Get path to UI file which should be in the "resource" folder of this package
        # TODO - hardcoded tt references
        ui_file = rospkg.RosPack().get_path('tt_gui') + '/resource/' + 'tt_panels_ui.ui'
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)

        self.layout = QVBoxLayout()

        self.tabs_widget = QStackedWidget()

        self.nav_widget = tt_panel_nav_ui(parent = self)
        self.mode_widget = tt_panel_mode_ui(parent = self)
        self.camera_widget = tt_panel_camera_ui(parent = self)
        self.systems_widget = tt_panel_systems_ui(parent = self)

        self.tabs_widget.addWidget(self.nav_widget)
        self.tabs_widget.addWidget(self.mode_widget)
        self.tabs_widget.addWidget(self.camera_widget)
        self.tabs_widget.addWidget(self.systems_widget)

        self.selection = 0

        self.button_widget = QWidget()
        self.button_layout = QHBoxLayout()

        self.button_nav.setStyleSheet(style_active)
        self.button_mode.setStyleSheet(style_normal)
        self.button_camera.setStyleSheet(style_normal)
        self.button_systems.setStyleSheet(style_normal)

        # Connect buttons to functions
        self.button_nav.clicked.connect(self.nav_button_pressed)
        self.button_mode.clicked.connect(self.mode_button_pressed)
        self.button_camera.clicked.connect(self.camera_button_pressed)
        self.button_systems.clicked.connect(self.systems_button_pressed)

        self.button_layout.addWidget(self.button_nav)
        self.button_layout.addWidget(self.button_mode)
        self.button_layout.addWidget(self.button_camera)
        self.button_layout.addWidget(self.button_systems)
        
        self.button_widget.setLayout(self.button_layout)

        self.layout.addWidget(self.tabs_widget)
        self.layout.addWidget(self.button_widget)

        self.setLayout(self.layout)

        self._update_button_state()

    def nav_button_pressed(self):
        self.selection = 0
        self._update_button_state()

    def mode_button_pressed(self):
        self.selection = 1
        self._update_button_state()

    def camera_button_pressed(self):
        self.selection = 2
        self._update_button_state()

    def systems_button_pressed(self):
        self.selection = 3
        self._update_button_state()

    def _update_button_state(self):
        self.tabs_widget.setCurrentIndex(self.selection)
        if self.selection == 0:
            self.button_nav.setStyleSheet(style_active)
            self.button_mode.setStyleSheet(style_normal)
            self.button_camera.setStyleSheet(style_normal)
            self.button_systems.setStyleSheet(style_normal)
        elif self.selection == 1:
            self.button_nav.setStyleSheet(style_normal)
            self.button_mode.setStyleSheet(style_active)
            self.button_camera.setStyleSheet(style_normal)
            self.button_systems.setStyleSheet(style_normal)
        elif self.selection == 2:
            self.button_nav.setStyleSheet(style_normal)
            self.button_mode.setStyleSheet(style_normal)
            self.button_camera.setStyleSheet(style_active)
            self.button_systems.setStyleSheet(style_normal)
        else:
            self.button_nav.setStyleSheet(style_normal)
            self.button_mode.setStyleSheet(style_normal)
            self.button_camera.setStyleSheet(style_normal)
            self.button_systems.setStyleSheet(style_active)
