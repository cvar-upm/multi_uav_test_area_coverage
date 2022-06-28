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
        