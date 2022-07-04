#!/usr/bin/env python

from PySide6.QtCore import Qt, Signal, QPointF
from PySide6.QtGui import QPen, QPixmap, QBrush, QPolygonF, QColor, QPainterPath
from PySide6.QtWidgets import (QGraphicsLineItem, QGraphicsPolygonItem, QGraphicsScene, QGraphicsView, 
QGraphicsItemGroup)
from performance_viewer.Drone import Drone
from mutac_msgs.msg import State, Plan, Identifier, Label
from geometry_msgs.msg import Pose
import numpy as np
import rospy
import yaml
import re
import os
import sys

class Map(QGraphicsView):
    # Signals
    trjSignal = Signal(Plan)
    posSignal = Signal(Drone)

    droneLostSignal = Signal(State)
    droneStatusSignal = Signal(State)

    showPlanSignal = Signal(bool)
    showPlanLeftSignal = Signal(bool)
    showRealSignal = Signal(bool)
    showRegionSignal = Signal(bool)
    showInspectedSignal = Signal(bool)

    timerSignal = Signal()
    wpSignal = Signal(Identifier)
    
    def __init__(self, parent=None):
        super().__init__(parent)

        # Init
        self._init_config()
        self._init_att()
        self._init_view()
        self._connect_signals()

        # Get coordinate conversion functions
        self.fx, self.fy = self.transfom_coordinates()

        # Paint region to inspect
        self.paint_region_limits(self.regions)

        # Initialize ROS node and subscribe to topics
        self.subscribe()

    # ------------------------
    # Paint and show functions
    # ------------------------

    # Paints the planned trajectory of the drone
    def paint_plan(self, data):
        for path in data.paths:
            droneID = path.identifier.natural
            path = path.points

            # Check in case of receiving a trajectory for a drone that is not in the list
            if droneID > len(self.drones) - 1: 
                print("WARNING: Trajectory received for a drone that doesn't exists.")
                return

            # If a drone planned trajectory changed, remove it to add the new one
            if self.drones[droneID].plan is not None:
                pos = self.drones[droneID].position
                path[0].point.x = pos[0]
                path[0].point.y = pos[1]
                self.scene.removeItem(self.drones[droneID].plan)
                self.scene.removeItem(self.drones[droneID].planLeft)

            # Set waypoints
            self.drones[droneID].set_wps(path)

            # Set pen color to the one assigned to the drone
            pen = QPen()
            pen.setWidth(2)
            pen.setColor(self.planned_colors[droneID % 6])
            
            # Creates new GraphicItemGroup for the trajectory
            trajectory = QGraphicsItemGroup()
            trajectory_left = QGraphicsItemGroup()
            trajectory.setZValue(1)
            trajectory_left.setZValue(1)

            # For each 2 points in the trajectory create a line between them and add it to the group
            for i in range(len(path) - 1):
                # Get the scene position
                start = self.get_position(path[i].point.x, path[i].point.y)
                end = self.get_position(path[i+1].point.x, path[i+1].point.y)

                line = QGraphicsLineItem(start[0], start[1], end[0], end[1])
                line.setPen(pen)
                line_left = QGraphicsLineItem(start[0], start[1], end[0], end[1])
                line_left.setPen(pen)

                trajectory.addToGroup(line)
                trajectory_left.addToGroup(line_left)

            if not self.show_trj: trajectory.hide()
            if not self.show_trj_left: trajectory_left.hide()

            self.scene.addItem(trajectory)
            self.scene.addItem(trajectory_left)
            self.drones[droneID].plan = trajectory
            self.drones[droneID].planLeft = trajectory_left
            self.drones[droneID].in_mission = True

    # Paints the plan that is left to cover
    def paint_plan_left(self):
        for drone in self.drones:
            if not drone.lost and drone.planLeft != None and drone.realTrj != None:
                self.scene.removeItem(drone.planLeft)

                modPlan = QGraphicsItemGroup()
                for i, item in enumerate(drone.planLeft.childItems()):
                    if i == 0:
                        pen = QPen()
                        pen.setWidth(2)
                        pen.setColor(self.planned_colors[drone.id % 6])
                        line = item.line()

                        pos = np.array(drone.position)
                        p1 = np.array(drone.lastWP)
                        p2 = np.array(drone.wp)

                        point = self.point_projection(pos, p1, p2)
                        point = self.get_position(point[0], point[1])

                        item = QGraphicsLineItem(point[0], point[1], line.p2().x(), line.p2().y())
                        item.setPen(pen)
                    
                    modPlan.addToGroup(item)
                
                if not self.show_trj_left: modPlan.hide() 
                self.scene.addItem(modPlan)
                drone.planLeft = modPlan

    # Paints the real trajectory the drone is following
    def paint_real_trj(self, drone):
        # Set pen color to the one assigned to the drone
        pen = QPen()
        pen.setWidth(2)
        pen.setColor(self.real_colors[drone.id % 6])
        
        inspection_color = QColor(self.real_colors[drone.id % 6])
        inspection_color.setAlpha(80)
        
        # Get the scene position
        pos = self.get_position(drone.position[0], drone.position[1])
        
        # Get the trajectory
        trajectory = drone.realTrj
        
        # If the drone just started, create the drone.
        # If not, move the drone and create a line between the current position aand the new one
        if drone.lastPosition:
            line = QGraphicsLineItem(drone.lastPosition[0], drone.lastPosition[1], pos[0], pos[1])
            line.setPen(pen)

            trajectory.addToGroup(line)
            
            drone.droneImg.prepareGeometryChange()
            drone.droneImg.setPos(pos[0] - 15, pos[1] - 10)
            
            # Add new point to the inspected region
            if drone.in_mission and not drone.lost:
                pen.setColor(inspection_color)
                pen.setWidth(10)

                if drone.lastLabel != drone.wpLabel and drone.id in self.inspected_regions:
                    _, path_item, path = self.inspected_regions[drone.id]
                    self.inspected_regions[drone.id] = (-1, path_item, path)

                elif drone.wpLabel == Label.COVERING_LABEL:
                    if drone.id not in self.inspected_regions:
                        path = QPainterPath()
                        path.moveTo(pos[0], pos[1])
                        path_item = self.scene.addPath(path, pen)
                        path_item.setZValue(1)

                        self.inspected_regions[drone.id] = (0, [path_item], [path])
                        if not self.show_inspected: path_item.hide()

                    else:
                        index, item_list, path_list = self.inspected_regions[drone.id]

                        if index < 0:
                            path = QPainterPath()
                            path.moveTo(pos[0], pos[1])
                            path_item = self.scene.addPath(path, pen)
                            path_item.setZValue(1)

                            item_list.append(path_item)
                            path_list.append(path)

                            self.inspected_regions[drone.id] = (len(item_list)-1, item_list, path_list)
                            if not self.show_inspected: path_item.hide()
                        else:
                            path_item = item_list[index]
                            path = path_list[index]

                            path.lineTo(pos[0], pos[1])
                            path_item.setPath(path)

        else:
            # Add position to real trajectory
            drone.droneImg.setPos(pos[0], pos[1])
            self.scene.addItem(drone.droneImg)

            trajectory = QGraphicsItemGroup()
            self.scene.addItem(trajectory)
            trajectory.setZValue(2)
            drone.realTrj = trajectory

            if not self.show_real_trj: trajectory.hide()
        
        drone.lastPosition = pos

    # Shows or hides the planned trajectory
    def show_plan(self, show):
        self.show_trj = show
        for drone in self.drones:
            if drone.plan is not None:
                drone.plan.show() if show else drone.plan.hide()

    # Shows or hides the plan that is left to cover
    def show_plan_left(self, show):
        self.show_trj_left = show
        for drone in self.drones:
            if drone.planLeft is not None:
                drone.planLeft.show() if show else drone.planLeft.hide()

    # Shows or hides the real trajectory
    def show_real(self, show):
        self.show_real_trj = show
        for drone in self.drones:
            if drone.realTrj is not None:
                drone.realTrj.show() if show else drone.realTrj.hide()

    # Paints region limits
    def paint_region_limits(self, regions):
        # Set pen and brush color
        color = QColor(234, 236, 238, 50)
        pen, brush = QPen(), QBrush()
        pen.setWidth(3)
        pen.setColor(color)
        brush.setColor(color)
        brush.setStyle(Qt.SolidPattern)
        
        # Draw the limits for each region
        for region in regions:
            # Creates a polygon with the different points
            polygon = QPolygonF()

            for point in region:
                pos = self.get_position(point[0], point[1])
                polygon.append(QPointF(pos[0], pos[1]))

            polygonItem = QGraphicsPolygonItem()
            polygonItem.setPolygon(polygon)
            polygonItem.setPen(pen)
            polygonItem.setBrush(brush)

            # Add polygon to the scene
            self.scene.addItem(polygonItem)
            polygonItem.hide()
            self.region_limits.append(polygonItem)

    # Shows or hides the inspection region limits
    def show_region_limits(self, show):
        for region in self.region_limits:
            region.show() if show else region.hide()

    # Shows or hides the drone region limits
    def show_inspected_regions(self, show):
        self.show_inspected = show
        for val in self.inspected_regions.values():
            for item in val[1]:
                item.show() if show else item.hide()

    # ------------------------
    # ROS functions
    # ------------------------

    def subscribe(self):
        # Planned trajectories topic
        rospy.Subscriber("planned_paths", Plan, self.trajectory_callback)

        # Lost drones topic
        rospy.Subscriber("drone_events", State, self.state_callback)

        # Waypoints topic
        rospy.Subscriber("covered_points", Identifier, self.wps_callback)

        # Current position topics
        for i in range(0, self.n_drones):
            topic = "drone{}/drone_pose".format(i+1)
            rospy.Subscriber(topic, Pose, self.drones[i].position_callback)

    def trajectory_callback(self, data):
        self.trjSignal.emit(data)
    
    def state_callback(self, data):
        if (data.state == State.LOST):
            self.droneLostSignal.emit(data)
        elif (data.state == State.FINISHED):
            self.droneStatusSignal.emit(data)

    def wps_callback(self, data):
        self.wpSignal.emit(data)

    def change_lost(self, data):
        self.drones[data.identifier.natural].lost = True
        self.drones[data.identifier.natural].change_title()
        self.scene.removeItem(self.drones[data.identifier.natural].plan)
        self.scene.removeItem(self.drones[data.identifier.natural].planLeft)
    
    def update_mission_status(self, data):
        self.drones[data.identifier.natural].in_mission = False
        self.update_wps(data.identifier)

    def update_wps(self, data):
        drone = self.drones[data.natural]
        drone.advanceWP()
        self.scene.removeItem(drone.planLeft.childItems()[0])

    def timer_callback(self, timer):
        self.timerSignal.emit()

    # ------------------------
    # Auxiliar functions
    # ------------------------

    # Calculates relation between the scene and real coordinates
    def transfom_coordinates(self):
        x_slope = self.scene.width() / (self.bottom_coord[0] - self.top_coord[0])
        y_slope = self.scene.height() / (self.bottom_coord[1] - self.top_coord[1])
        x_intercept = - x_slope * self.top_coord[0]
        y_intercept = - y_slope * self.top_coord[1]

        return [x_slope, x_intercept], [y_slope, y_intercept]

    # Converts real coordinates into scene coordinates
    def get_position(self, x, y):
        return [self.fx[0] * x + self.fx[1], self.fy[0] * y + self.fy[1]]

    # Calculate projection of the position in the path
    def point_projection(self, pos, wp1, wp2):
        # OA + ((AM . u) / |u|^2) x u
        AM = pos - wp1
        u = wp2 - wp1

        scalar = np.dot(AM, u) / pow(np.linalg.norm(u), 2)
        return wp1 + scalar * u

    # ------------------------
    # Init functions
    # ------------------------

    def _init_att(self):
        self.planned_colors = [Qt.red, Qt.green, Qt.magenta, Qt.yellow, Qt.cyan, Qt.blue]
        self.real_colors = [Qt.darkRed, Qt.darkGreen, Qt.darkMagenta, Qt.darkYellow, Qt.darkCyan, Qt.darkBlue]

        self.show_trj, self.show_trj_left, self.show_real_trj, self.show_inspected = False, True, True, False

        # Initialize n drones
        self.drones = []
        for i in range(0, self.n_drones):
            self.drones.append(Drone(self, i))

        # Creates new items group for the inspected regions
        self.drone_limits = QGraphicsItemGroup()
        self.drone_limits.setZValue(1)
        self.drone_limits.hide()

        self.inspected_regions = dict()
        self.region_limits = []

        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

    def _init_config(self):
        config_path = rospy.get_param(rospy.get_name() + "/config")
        
        self.path_matcher = re.compile(r'\$\{([^}^{]+)\}')
        yaml.add_implicit_resolver('!path',self.path_matcher)
        yaml.add_constructor('!path', self.path_constructor)

        try:
            with open(config_path, 'r') as file:
                conf = yaml.load(file, Loader=yaml.FullLoader)
        except OSError:
            print("\033[31;1mERROR: Cannot open file:\033[0m", config_path, file=sys.stderr)
            sys.exit(1)
            
        # Get config parameters
        self.top_coord = conf["img_coordinates"]["top"]
        self.bottom_coord = conf["img_coordinates"]["bottom"]
        
        try:
            self.n_drones = rospy.get_param("/mutac/n_drones")
        except KeyError:
            print("\033[31;1mERROR: Missing parameter 'n_drones'.\033[0m", file=sys.stderr)
            sys.exit(1)

        self.regions = []
        for reg in conf["regions"].values():
            self.regions.append(reg)

        # Set map image
        if os.path.exists(conf["img_path"]):
            self.image = QPixmap(conf["img_path"])
            self.setFixedSize(self.image.width(), self.image.height())
        else:
            print("\033[31;1mERROR: File not found:\033[0m", conf["img_path"])
            sys.exit()
 
    def _init_view(self):
        # Set view size
        self.setFixedSize(self.image.width(), self.image.height())

        # Set scene
        self.scene = QGraphicsScene()
        self.scene.setSceneRect(self.rect())
        self.scene.setItemIndexMethod(QGraphicsScene.NoIndex)

        # Add drone limits to the scene
        self.scene.addItem(self.drone_limits)

        # Set view
        self.setScene(self.scene)
        self.setBackgroundBrush(QBrush(self.image))
        self.setCacheMode(QGraphicsView.CacheBackground)
        self.setViewportUpdateMode(QGraphicsView.BoundingRectViewportUpdate)
        
        rcontent = self.contentsRect()
        self.setSceneRect(0, 0, rcontent.width(), rcontent.height())

    def _connect_signals(self):
        self.trjSignal.connect(self.paint_plan)
        self.posSignal.connect(self.paint_real_trj)

        self.droneLostSignal.connect(self.change_lost)
        self.droneStatusSignal.connect(self.update_mission_status)

        self.showPlanSignal.connect(self.show_plan)
        self.showPlanLeftSignal.connect(self.show_plan_left)
        self.showRealSignal.connect(self.show_real)
        self.showRegionSignal.connect(self.show_region_limits)
        self.showInspectedSignal.connect(self.show_inspected_regions)

        self.timerSignal.connect(self.paint_plan_left)
        self.wpSignal.connect(self.update_wps)

    # --------------------------------
    # YAML path environment vairables
    # --------------------------------

    def path_constructor(self, loader, node):
        value = node.value
        match = self.path_matcher.match(value)
        env_var = match.group()[2:-1]
        return os.environ.get(env_var) + value[match.end():]
