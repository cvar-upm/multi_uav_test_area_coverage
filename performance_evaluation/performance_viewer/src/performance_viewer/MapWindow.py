###################################################################################
# \authors   Elisa Martinez-Abad
# \copyright Copyright (c) 2022 Universidad Politecnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###################################################################################

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
