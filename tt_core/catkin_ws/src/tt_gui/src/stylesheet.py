from python_qt_binding.QtWidgets import QDesktopWidget

import os

desktop = QDesktopWidget().availableGeometry()

START_X = desktop.x()
START_Y = desktop.y()
TOTAL_W = desktop.width()
TOTAL_H = desktop.height()

NO_STYLE      = ''
BUTTON_NORMAL = 'color:black; font-size:26px; background-color:rgb(200,200,200)'
BUTTON_ACTIVE = 'color:green; font-size:26px; background-color:rgb(200,200,200)'
BUTTON_OFF    = 'color:black; font-size:26px; background-color:rgb(230,230,230)'

SMALL_BUTTON_NORMAL = 'color:black; font-size:15px; background-color:rgb(200,200,200)'

FRAME_STYLE = 'margin:0px; border:1px solid rgb(0, 0, 0);'
LABEL_STYLE = 'margin:0px; border:1px solid rgb(0, 0, 0); color:black; font-size:26px;'
SUB_L_STYLE = 'margin:0px; border:1px solid rgb(0, 0, 0); color:black; font-size:20px;'
MES_L_STYLE = 'margin:0px; border:1px solid rgb(0, 0, 0); color:black; font-size:15px;'

S_DEGRADED  = 'color:black; font-size:15px; background-color:rgb(200,125,50);'
S_RUNNING   = 'color:black; font-size:15px; background-color:rgb(20,200,40);'
S_DISABLED  = 'color:black; font-size:15px; background-color:rgb(200,50,20);'

MAP_WEIGHT = 7
PAN_WEIGHT = 3

PAN_DIAG_WEIGHT = 9
PAN_SWITCH_WEIGHT = 1

STATUS_LABELS = ['Degraded', 'Running', 'Disabled']
TOGGLE_LABELS = ['Live Feed', 'Pause']
CONTROL_LABELS = ['Disable', 'Enable']

TRACK_SYSTEM_STATUS = 1
FREQ_CUTOFF  = 1

MAP_PATH = os.environ['TT_ROOT'] + '/tt_core/catkin_ws/src/tt_gui/imgs/rit_osm_map.png'
LOGO_PATH = os.environ['TT_ROOT'] + '/tt_core/catkin_ws/src/tt_gui/imgs/rit_tt.png'

MIN_LAT = 43.0898
MAX_LAT = 43.0776
MIN_LON = -77.6873
MAX_LON = -77.6640

PIX_W = 9771
PIX_H = 7046

LOGO_SIZE = 30
