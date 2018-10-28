from python_qt_binding.QtWidgets import QDesktopWidget

desktop = QDesktopWidget().availableGeometry()

START_X = desktop.x()
START_Y = desktop.y()
TOTAL_W = desktop.width()
TOTAL_H = desktop.height()

BUTTON_NORMAL = 'color:black; font-size: 26px; background-color: rgb(200,200,200)'
BUTTON_ACTIVE = 'color:green; font-size: 26px; background-color: rgb(200,200,200)'

FRAME_STYLE = 'margin:0px; border:1px solid rgb(0, 0, 0);'
LABEL_STYLE = 'margin:0px; border:1px solid rgb(0, 0, 0); color: black; font-size: 26px;'

MAP_WEIGHT = 7
PAN_WEIGHT = 3

PAN_DIAG_WEIGHT = 9
PAN_SWITCH_WEIGHT = 1
