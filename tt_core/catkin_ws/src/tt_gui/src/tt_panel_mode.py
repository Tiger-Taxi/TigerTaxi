from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from PyQt5.QtGui import *

from argparse import ArgumentParser

from std_msgs.msg import Float32
from playsound import playsound

import rospkg
import rospy
import os

from stylesheet import *

# Widget for the control mode panel
class tt_panel_mode_ui(QWidget):
    def __init__(self, parent):
        super(tt_panel_mode_ui, self).__init__()
        self.setParent(parent)
        self.setObjectName('tt_panel_mode_ui')
        ui_file = rospkg.RosPack().get_path('tt_gui') + '/resource/' + 'tt_panel_mode_ui.ui'
        loadUi(ui_file, self)

        self.mode_control_frame.setStyleSheet(FRAME_STYLE)
        self.mode_control_label.setStyleSheet(LABEL_STYLE)

        self.button_man.setStyleSheet(BUTTON_NORMAL)
        self.button_rem.setStyleSheet(BUTTON_NORMAL)
        self.button_aut.setStyleSheet(BUTTON_NORMAL)

        self.mode_control_layout = QVBoxLayout()
        self.mode_control_layout.addWidget(self.mode_control_label)
        self.mode_control_layout.addWidget(self.button_man)
        self.mode_control_layout.addWidget(self.button_rem)
        self.mode_control_layout.addWidget(self.button_aut)
        self.mode_control_frame.setLayout(self.mode_control_layout)

        self.sound_control_frame.setStyleSheet(FRAME_STYLE)
        self.sound_control_label.setStyleSheet(LABEL_STYLE)

        self.button_sound1.setStyleSheet(BUTTON_NORMAL)
        self.button_sound2.setStyleSheet(BUTTON_NORMAL)
        self.button_sound3.setStyleSheet(BUTTON_NORMAL)
        self.button_sound4.setStyleSheet(BUTTON_NORMAL)

        self.button_man.clicked.connect(self.button_man_pressed)
        self.button_rem.clicked.connect(self.button_rem_pressed)
        self.button_aut.clicked.connect(self.button_aut_pressed)

        self.sound_control_layout = QVBoxLayout()
        self.sound_control_layout.addWidget(self.sound_control_label)
        self.sound_control_layout.addWidget(self.button_sound1)
        self.sound_control_layout.addWidget(self.button_sound2)
        self.sound_control_layout.addWidget(self.button_sound3)
        self.sound_control_layout.addWidget(self.button_sound4)
        self.sound_control_frame.setLayout(self.sound_control_layout)

        self.button_sound1.clicked.connect(self.button_sound1_pressed)
        self.button_sound2.clicked.connect(self.button_sound2_pressed)
        self.button_sound3.clicked.connect(self.button_sound3_pressed)
        self.button_sound4.clicked.connect(self.button_sound4_pressed)

        self.layout = QVBoxLayout()
        self.layout.addStretch()
        self.layout.addWidget(self.mode_control_frame)
        self.layout.addStretch()
        self.layout.addWidget(self.sound_control_frame)
        self.layout.addStretch()
        self.setLayout(self.layout)

        self.publisher_controlmode = rospy.Publisher('apm_controlmode', Float32, queue_size = 1)

        self.controlmode = 0
        self._update_button_state()

    def button_man_pressed(self):
        self.controlmode = 0
        self._update_button_state()
        self._publish_mode(None)

    def button_rem_pressed(self):
        self.controlmode = 1
        self._update_button_state()
        self._publish_mode(None)

    def button_aut_pressed(self):
        self.controlmode = 2
        self._update_button_state()
        self._publish_mode(None)

    def button_sound1_pressed(self):
        playsound(os.environ['TT_ROOT'] + '/horn_sounds/DJ Airhorn Sound Effect.mp3')

    def button_sound2_pressed(self):
        playsound(os.environ['TT_ROOT'] + '/horn_sounds/Tiger Roar.mp3')

    def button_sound3_pressed(self):
        playsound(os.environ['TT_ROOT'] + '/horn_sounds/Fountains of Wayne - Stacy\'s Mom.mp3')

    def button_sound4_pressed(self):
        playsound(os.environ['TT_ROOT'] + '/horn_sounds/dixie-horn_daniel-simion.mp3')

    def _update_button_state(self):
        styles = [BUTTON_NORMAL] * 3
        styles[self.controlmode] = BUTTON_ACTIVE

        self.button_man.setStyleSheet(styles[0])
        self.button_rem.setStyleSheet(styles[1])
        self.button_aut.setStyleSheet(styles[2])

    def _publish_mode(self, event):
        self.publisher_controlmode.publish(Float32(self.controlmode))
