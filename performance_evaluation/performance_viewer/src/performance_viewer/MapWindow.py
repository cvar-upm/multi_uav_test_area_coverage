#!/usr/bin/env python
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QWidget, QHBoxLayout
from performance_viewer.Map import Map

class MapWindow(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.map = Map(self)
        layout = QHBoxLayout()
        layout.addWidget(self.map, Qt.AlignCenter)
        self.setLayout(layout)
        
        self.ratio = self.map.height() / self.map.width()
        self.start = True

    # Resize map
    def resizeEvent(self, event):
        super().resizeEvent(event)

        if self.start: 
            self.start = False
            return

        ratioW = event.size().width() / self.map.image.width() 
        ratioH = event.size().height() / self.map.image.height() 

        ratio = ratioW if ratioW < ratioH else ratioH

        newW = self.map.image.width() * ratio
        newH = self.map.image.height() * ratio

        self.map.setFixedSize(newW, newH)
        self.map.fitInView(self.map.scene.sceneRect(), Qt.KeepAspectRatio)
