from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from PyQt5.QtGui import *

from argparse import ArgumentParser

import rospkg
import rospy
import os

from tt_panel_nav import tt_panel_nav_ui
from tt_panel_mode import tt_panel_mode_ui
from tt_panel_camera import tt_panel_camera_ui
from tt_panel_systems import tt_panel_systems_ui

from stylesheet import *

# Widget to hold all panels and selection mechanism
class tt_panels_ui(QWidget):
    def __init__(self):
        super(tt_panels_ui, self).__init__()
        self.setObjectName('tt_panels_ui')
        ui_file = rospkg.RosPack().get_path('tt_gui') + '/resource/' + 'tt_panels_ui.ui'
        loadUi(ui_file, self)

        self.tab_frame = QFrame(parent = self)

        self.nav_widget = tt_panel_nav_ui(parent = self)
        self.mode_widget = tt_panel_mode_ui(parent = self)
        #self.camera_widget = tt_panel_camera_ui(parent = self)
        self.systems_scroll.setWidgetResizable(True)
        self.systems_widget = tt_panel_systems_ui(parent = self)
        self.systems_scroll.setWidget(self.systems_widget)
        self.systems_scroll.verticalScrollBar().setStyleSheet(VERT_SCROLL)

        self.tab_layout = QStackedLayout()
        self.tab_layout.addWidget(self.nav_widget)
        self.tab_layout.addWidget(self.mode_widget)
        #self.tab_layout.addWidget(self.camera_widget)
        self.tab_layout.addWidget(self.systems_scroll)
        self.tab_frame.setLayout(self.tab_layout)

        self.button_frame = QFrame(parent = self)

        self.button_nav.clicked.connect(self.nav_button_pressed)
        self.button_mode.clicked.connect(self.mode_button_pressed)
        #self.button_camera.clicked.connect(self.camera_button_pressed)
        self.button_systems.clicked.connect(self.systems_button_pressed)

        self.button_layout = QHBoxLayout()
        self.button_layout.addWidget(self.button_nav)
        self.button_layout.addWidget(self.button_mode)
        #self.button_layout.addWidget(self.button_camera)
        self.button_layout.addWidget(self.button_systems)
        self.button_frame.setLayout(self.button_layout)

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.tab_frame, PAN_MAP_WEIGHT)
        self.layout.addWidget(self.button_frame, PAN_SWITCH_WEIGHT)
        self.setLayout(self.layout)

        self.tab_frame.setStyleSheet(FRAME_STYLE)
        self.button_frame.setStyleSheet(FRAME_STYLE)

        self.selection = 0
        self._update_button_state()

    def nav_button_pressed(self):
        self.selection = 0
        self._update_button_state()

    def mode_button_pressed(self):
        self.selection = 1
        self._update_button_state()

#    def camera_button_pressed(self):
#        self.selection = 2
#        self._update_button_state()

    def systems_button_pressed(self):
        self.selection = 2
        self._update_button_state()

    def _update_button_state(self):
        self.tab_layout.setCurrentIndex(self.selection)
        styles = [BUTTON_NORMAL] * 3
        styles[self.selection] = BUTTON_ACTIVE

        self.button_nav.setStyleSheet(styles[0])
        self.button_mode.setStyleSheet(styles[1])
        #self.button_camera.setStyleSheet(styles[2])
        self.button_systems.setStyleSheet(styles[2])
