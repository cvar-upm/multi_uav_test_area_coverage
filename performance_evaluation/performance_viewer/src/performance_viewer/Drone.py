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

from PySide6.QtWidgets import QGraphicsPixmapItem, QGraphicsTextItem, QGraphicsItemGroup
from PySide6.QtGui import QPixmap
from PySide6.QtCore import Qt
from mutac_msgs.msg import Label
import rospy

class Drone:
    def __init__(self, map, id=1):
        self.id = id
        self.map = map
        self.plan, self.realTrj, self.planLeft =  None, None, None
        self.waypoints = []
        self.position = [0,0,0]
        self.lastPosition = []
        self.in_mission, self.lost = False, False
        self.wp, self.lastWP = [], []
        self.wpLabel, self.lastLabel = Label.POSITIONING_LABEL, Label.POSITIONING_LABEL

        self._init_image()

    def position_callback(self, msg):
        self.position = [msg.position.x, msg.position.y, msg.position.z]
        self.map.posSignal.emit(self)

    def set_wps(self, path):
        self.waypoints = list(path)
        
        wp = self.waypoints.pop(0)
        self.lastLabel = wp.label.natural

        wp = self.waypoints[0]
        self.wp = [wp.point.x, wp.point.y, wp.point.z]
        self.lastWP = self.position
        self.wpLabel = wp.label.natural

    def advanceWP(self):
        if len(self.waypoints) > 1:
            self.waypoints.pop(0)
            wp = self.waypoints[0]

            self.lastWP = self.wp
            self.wp = [wp.point.x, wp.point.y, wp.point.z]
            
            self.lastLabel = self.wpLabel
            self.wpLabel = wp.label.natural

    def change_title(self):
        self.text.setHtml('<div style="background-color:rgba(0, 0, 0, 0.3)">UAV {} <span style="color:rgb(255, 255, 0)">(lost)</span></p>'.format(self.id+1))
    
    def _init_image(self):
        img_path = rospy.get_param(rospy.get_name() + "/drone_img")
        imgOb = QGraphicsPixmapItem(QPixmap(img_path))
        imgOb.setScale(0.05)

        self.text = QGraphicsTextItem()
        self.text.setDefaultTextColor(Qt.white)
        self.text.setHtml('<div style="background-color:rgba(0, 0, 0, 0.3)">UAV {}</p>'.format(self.id+1))
        self.text.setPos(-12, -22)

        self.droneImg = QGraphicsItemGroup()
        self.droneImg.setZValue(3)
        self.droneImg.addToGroup(imgOb)
        self.droneImg.addToGroup(self.text)
        