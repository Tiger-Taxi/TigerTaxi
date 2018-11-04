from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from argparse import ArgumentParser

from std_msgs.msg import Float32
from std_msgs.msg import String

import numpy as np
import rospkg
import rospy
import os

from stylesheet import *

class tt_map_ui(QWidget):
    def __init__(self):
        super(tt_map_ui, self).__init__()
        self.setObjectName('tt_map_ui')
        ui_file = rospkg.RosPack().get_path('tt_gui') + '/resource/' + 'tt_map_ui.ui'
        loadUi(ui_file, self)
        self.viewer = MapViewer(self)
        self.viewer.photoClicked.connect(self.photoClicked)
        # Arrange layout
        VBlayout = QVBoxLayout(self)
        VBlayout.addWidget(self.viewer)

        self.gps_coords_lat = np.repeat(np.expand_dims(np.arange(MIN_LAT, MAX_LAT, PIX_H), axis = 0), PIX_W, axis = 1)
        self.gps_coords_lon = np.repeat(np.expand_dims(np.arange(MIN_LON, MAX_LON, PIX_W), axis = 0), PIX_H, axis = 0)

    def photoClicked(self, pos):
    	rospy.loginfo(pos)
    	img_coords = self.viewer._photo.mapToItem(self.viewer._photo, self.viewer.mapToScene(pos))
    	rospy.loginfo(img_coords)
        self.viewer._logo.setOffset(int(img_coords.x() - (LOGO_SIZE / 2)), int(img_coords.y() - (LOGO_SIZE / 2)))

class MapViewer(QGraphicsView):
    photoClicked = pyqtSignal(QPoint)

    def __init__(self, parent):
        super(MapViewer, self).__init__(parent)
        self._zoom = 0
        self._scene = QGraphicsScene(self)
        self._photo = QGraphicsPixmapItem()
        self._logo = QGraphicsPixmapItem()

        self._photo.setPixmap(QPixmap(MAP_PATH))
        self._logo.setPixmap(QPixmap(LOGO_PATH).scaledToWidth(LOGO_SIZE).scaledToHeight(LOGO_SIZE))

        self._scene.addItem(self._photo)
        self._scene.addItem(self._logo)
        self.setScene(self._scene)

        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.fitInView()

    def fitInView(self, scale=True):
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

        if self._zoom > 0:
            self.scale(factor, factor)
        elif self._zoom == 0:
            self.fitInView()
        else:
            self._zoom = 0

    def toggleDragMode(self):
        if self.dragMode() == QGraphicsView.ScrollHandDrag:
            self.setDragMode(QGraphicsView.NoDrag)
        elif not self._photo.pixmap().isNull():
            self.setDragMode(QGraphicsView.ScrollHandDrag)

    def mousePressEvent(self, event):
        if self._photo.isUnderMouse():
            self.photoClicked.emit(QPoint(event.pos()))
        super(MapViewer, self).mousePressEvent(event)