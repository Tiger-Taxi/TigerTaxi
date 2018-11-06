from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from argparse import ArgumentParser

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from std_msgs.msg import String

import numpy as np
import rospkg
import rospy
import os

from stylesheet import *

# Widget to hold map and pan/lock buttons
class tt_map_ui(QWidget):
    gpsSignal = pyqtSignal(QPoint)
    goalSignal = pyqtSignal(QPoint)

    def __init__(self):
        super(tt_map_ui, self).__init__()
        self.setObjectName('tt_map_ui')
        ui_file = rospkg.RosPack().get_path('tt_gui') + '/resource/' + 'tt_map_ui.ui'
        loadUi(ui_file, self)
        self.viewer = MapViewer(self)
        self.viewer.photoClicked.connect(self.photoClicked)
        self.gpsSignal.connect(self.updateLogoOffset)
        self.goalSignal.connect(self.updateGoalOffset)

        self.layout = QVBoxLayout(self)

        self.button_layout = QHBoxLayout()
        self.button_layout.addWidget(self.button_mode)
        self.button_layout.addWidget(self.button_lock)
        self.button_layout.addWidget(self.button_zoom_in)
        self.button_layout.addWidget(self.button_zoom_out)
        self.button_widget.setLayout(self.button_layout)

        self.layout.addWidget(self.viewer, PAN_MAP_WEIGHT)
        self.layout.addWidget(self.button_widget, PAN_SWITCH_WEIGHT)
        self.setLayout(self.layout)

        self.button_mode.setStyleSheet(BUTTON_NORMAL)
        self.button_lock.setStyleSheet(BUTTON_NORMAL)
        self.button_zoom_in.setStyleSheet(BUTTON_NORMAL)
        self.button_zoom_out.setStyleSheet(BUTTON_NORMAL)
        self.button_mode.clicked.connect(self.togglePanMode)
        self.button_lock.clicked.connect(self.toggleLockMode)
        self.button_zoom_in.clicked.connect(self.zoomIn)
        self.button_zoom_out.clicked.connect(self.zoomOut)

        self.button_widget.setStyleSheet(FRAME_STYLE)
        self.viewer.setStyleSheet(FRAME_STYLE)

        self.gps_lats = np.flip(np.linspace(MIN_LAT, MAX_LAT, PIX_H))
        self.gps_lons = np.linspace(MIN_LON, MAX_LON, PIX_W)
        self.subscriber_GPS = rospy.Subscriber('tt_gui/throttle/gps', NavSatFix, self.gps_callback, queue_size = QUEUE_SIZE)
        self.subscriber_nav = rospy.Subscriber('tt_gui/nav/status', Float32, self.nav_callback, queue_size = QUEUE_SIZE)
        self.publisher_nav = rospy.Publisher('tt_gui/nav/coords', String, queue_size = QUEUE_SIZE)

        self.pan = True
        self.lock = False
        self.status = 0

        self.goal_lat = None
        self.goal_lon = None

        coords = QPoint(-1000, -1000)
        self.gpsSignal.emit(coords)
        self.goalSignal.emit(coords)
        self.update()

    def gps_callback(self, data):
        lat = data.latitude
        lon = data.longitude

        if lat != 0 and lon != 0:
            y_pix, x_pix = self.getXYCoordsFromGps(lat, lon)
            coords = QPoint(x_pix - (LOGO_SIZE // 2), y_pix - (LOGO_SIZE // 2))
            self.gpsSignal.emit(coords)
        else:
            coords = QPoint(-1000, -1000)
            self.gpsSignal.emit(coords)

    def nav_callback(self, data):
        self.status = data.data
        if self.goal_lon != None and self.goal_lat != None and (self.status == 1 or self.status == 2):
            y_pix, x_pix = self.getXYCoordsFromGps(self.goal_lat, self.goal_lon)
            coords = QPoint(x_pix - (LOGO_SIZE // 2), y_pix - (LOGO_SIZE // 2))
        else:
            coords = QPoint(-1000, -1000)
        self.goalSignal.emit(coords)

    def getXYCoordsFromGps(self, lat, lon):
        y_pix = np.where(self.gps_lats > lat)[0][-1]
        x_pix = np.where(self.gps_lons < lon)[0][-1]
        return y_pix, x_pix

    def getGpsCoordsFromXY(self, x, y):
        lat = self.gps_lats[y]
        lon = self.gps_lons[x]
        return lat, lon

    def updateLogoOffset(self, pos):
        self.viewer._logo.setOffset(pos.x(), pos.y())
        if self.lock and pos.x() > 0 and pos.y() > 0:
            # TODO - leaves behind artifacts when zoomed in
            self.viewer.centerOn(self.viewer._logo)

    def updateGoalOffset(self, pos):
        self.viewer._goal.setOffset(pos.x(), pos.y())

    def togglePanMode(self):
        self.pan = not self.pan
        self.update()

    def toggleLockMode(self):
        self.lock = not self.lock
        self.update()

    def update(self):
        if not self.pan:
            self.viewer.setDragMode(QGraphicsView.NoDrag)
            self.button_mode.setText('Pan')
        else:
            self.viewer.setDragMode(QGraphicsView.ScrollHandDrag)
            self.button_mode.setText('Select')

        if self.lock:
            self.button_lock.setText('Unlock')
        else:
            self.button_lock.setText('Lock')

    def photoClicked(self, pos):
        if not self.pan:
            img_coords = self.viewer._photo.mapToItem(self.viewer._photo, self.viewer.mapToScene(pos))
            lat, lon = self.getGpsCoordsFromXY(int(img_coords.x()), int(img_coords.y()))
            self.goal_lat = lat
            self.goal_lon = lon
            msg = String(str(lat) + ',' + str(lon))
            self.publisher_nav.publish(msg)

    def zoomIn(self):
        self.viewer.zoomIn()

    def zoomOut(self):
        self.viewer.zoomOut()

# Qt object to hold the images
class MapViewer(QGraphicsView):
    photoClicked = pyqtSignal(QPoint)

    def __init__(self, parent):
        super(MapViewer, self).__init__(parent)
        self._zoom = 0
        self._scene = QGraphicsScene(self)
        self._photo = QGraphicsPixmapItem()
        self._logo = QGraphicsPixmapItem()
        self._goal = QGraphicsPixmapItem()

        self._photo.setPixmap(QPixmap(OSM_PATH))
        self._logo.setPixmap(QPixmap(LOGO_PATH).scaledToWidth(LOGO_SIZE).scaledToHeight(LOGO_SIZE))
        self._goal.setPixmap(QPixmap(GOAL_PATH).scaledToWidth(LOGO_SIZE).scaledToHeight(LOGO_SIZE))

        self._scene.addItem(self._photo)
        self._scene.addItem(self._logo)
        self._scene.addItem(self._goal)
        self.setScene(self._scene)

        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setDragMode(QGraphicsView.ScrollHandDrag)

    def fitInView(self, scale = True):
        rect = QRectF(self._photo.pixmap().rect())
        self.setSceneRect(rect)
        unity = self.transform().mapRect(QRectF(0, 0, 1, 1))
        self.scale(1 / unity.width(), 1 / unity.height())
        viewrect = self.viewport().rect()
        scenerect = self.transform().mapRect(rect)
        factor = min(viewrect.width() / scenerect.width(), viewrect.height() / scenerect.height())
        self.scale(factor, factor)
        self._zoom = 0

    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            factor = 1.25
            self._zoom += 1
        else:
            factor = 0.8
            self._zoom -= 1

        if self._zoom >= MAX_ZOOM:
            self._zoom = MAX_ZOOM
        elif self._zoom > 0:
            self.scale(factor, factor)
        else:
            self._zoom = 0
            self.fitInView()

    def zoomIn(self):
        factor = 1.25
        self._zoom += 1

        if self._zoom >= MAX_ZOOM:
            self._zoom = MAX_ZOOM
        else:
            self.scale(factor, factor)

    def zoomOut(self):
        factor = 0.8
        self._zoom -= 1

        if self._zoom > 0:
            self.scale(factor, factor)
        else:
            self._zoom = 0
            self.fitInView()

    def mousePressEvent(self, event):
        if self._photo.isUnderMouse():
            self.photoClicked.emit(QPoint(event.pos()))
        super(MapViewer, self).mousePressEvent(event)