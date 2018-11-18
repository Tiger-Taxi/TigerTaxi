# Style-sheets and global definitions for tt_gui

import os

############################################################################################################
# Style-sheets                                                                                             #
############################################################################################################

# Frame/widget style
FRAME_STYLE         = 'margin:0px; border:1px solid rgb(0, 0, 0);'

# Vertical scroll bar for subsystems panel
VERT_SCROLL         = 'width:20px; background-color:rgb(200,200,200)'

# Large button styles
BUTTON_NORMAL       = 'color:black; font-size:26px; background-color:rgb(200,200,200)'
BUTTON_ACTIVE       = 'color:green; font-size:26px; background-color:rgb(200,200,200)'
BUTTON_OFF          = 'color:black; font-size:26px; background-color:rgb(230,230,230)'

# Small button style
SMALL_BUTTON_NORMAL = 'color:black; font-size:15px; background-color:rgb(200,200,200)'

# Label styles
LABEL_STYLE         = 'margin:0px; border:1px solid rgb(0, 0, 0); color:black; font-size:26px;'
SUB_L_STYLE         = 'margin:0px; border:1px solid rgb(0, 0, 0); color:black; font-size:20px;'
MES_L_STYLE         = 'margin:0px; border:1px solid rgb(0, 0, 0); color:black; font-size:15px;'

# Status label styles
S_DEGRADED          = 'color:black; font-size:15px; background-color:rgb(200,125,50);'
S_RUNNING           = 'color:black; font-size:15px; background-color:rgb(20,200,40);'
S_DISABLED          = 'color:black; font-size:15px; background-color:rgb(200,50,20);'

############################################################################################################
# Global Definitions                                                                                       #
############################################################################################################

# Paths to images (map, tt image, goal image)
OSM_PATH            = os.environ['TT_ROOT'] + '/tt_core/catkin_ws/src/tt_gui/imgs/rit_osm_map.png'
LOGO_PATH           = os.environ['TT_ROOT'] + '/tt_core/catkin_ws/src/tt_gui/imgs/rit_tt.png'
GOAL_PATH           = os.environ['TT_ROOT'] + '/tt_core/catkin_ws/src/tt_gui/imgs/rit_dest.png'

# Labels for status state and message toggle state
STATUS_LABELS       = ['Degraded', 'Running', 'No Messages']
TOGGLE_LABELS       = ['Live Feed', 'Pause']

# Long/lat boundaries for the campus map
LAT_OFFSET          = 0.000045
LON_OFFSET          = -0.000033
MIN_LAT             = 43.0776
MAX_LAT             = 43.089845
MIN_LON             = -77.687333
MAX_LON             = -77.6640

# Pixel width and height of campus map
PIX_W               = 9771
PIX_H               = 7046

# Maximum zoom intensity for campus map
MAX_ZOOM            = 15

# Square pixel size for tt image and goal image
LOGO_SIZE           = 30

# Weight out of 10 for map and control panels
MAP_WEIGHT          = 7
PAN_WEIGHT          = 3

# Weight out of 20 for control panels and panel selection
PAN_MAP_WEIGHT      = 19
PAN_SWITCH_WEIGHT   = 1

# Switch to track the imu/gps messages and frequencies of all sensors
TRACK_SYSTEM_STATUS = 1

# Queue size for all message callbacks
QUEUE_SIZE          = 5

# Rate at which each sensor frequency label is updated
FREQ_UPDATE         = 1

# Lowest frequency acceptable to consider a sensor running as opposed to degraded
FREQ_CUTOFF         = 10
