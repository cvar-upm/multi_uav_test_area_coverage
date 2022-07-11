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

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QAction, QKeySequence, QIcon
from PySide6.QtWidgets import QMainWindow, QDockWidget, QTreeWidget, QTreeWidgetItem
from performance_viewer.MapWindow import MapWindow
from mutac_msgs.msg import Metrics
import rospy

class MainWindow(QMainWindow):
    metricsSignal = Signal(Metrics)

    def __init__(self):
        QMainWindow.__init__(self)
        rospy.init_node('performance_viewer')

        self.setWindowTitle("Performance viewer")
        img_path = rospy.get_param(rospy.get_name() + "/window_icon")
        self.setWindowIcon(QIcon(img_path))

        # Create widgets
        self._createActions()
        self._createMenu()
        self._createToolBar()
        self._createMetrics()
        self._createMap()

        # Connect signal
        self.metricsSignal.connect(self.change_metrics)

        # Subscribe to metrics
        self.subscribe()

        # Window dimensions
        geometry = self.screen().availableGeometry()
        self.setMinimumSize(geometry.width() * 0.5, geometry.height() * 0.4)
        self.resize(geometry.width(), geometry.height() * 0.7)

    # ------------------------
    # Widgets
    # ------------------------

    def _createMenu(self):
        # Menu
        self.menu = self.menuBar()

        # File menu
        self.file_menu = self.menu.addMenu("&File")
        self.file_menu.addAction(self.exit_action)

        # Map menu
        self.map_menu = self.menu.addMenu("&Map")
        self.map_menu.addAction(self.show_region_limits)
        self.map_menu.addAction(self.show_inspected_region)
        self.map_menu.addSeparator()
        self.map_menu.addAction(self.show_plan)
        self.map_menu.addAction(self.show_plan_left)
        self.map_menu.addAction(self.show_real_trj)

    def _createToolBar(self):
        # Map toolbar
        self.map_tb = self.addToolBar("&Map")
        self.map_tb.addAction(self.show_region_limits)
        self.map_tb.addAction(self.show_inspected_region)
        self.map_tb.addSeparator()
        self.map_tb.addAction(self.show_plan)
        self.map_tb.addAction(self.show_plan_left)
        self.map_tb.addAction(self.show_real_trj)

    def _createMetrics(self):
        n_drones = rospy.get_param("/mutac/n_drones", 1)

        self.drone_list = QTreeWidget()
        self.drone_list.setColumnCount(2)
        self.drone_list.setHeaderLabels(["Parameters", "Values"])
        
        items = []

        item = QTreeWidgetItem(["Multi-UAV System"])
        item.addChild(QTreeWidgetItem(["Mission time", "-s"]))
        item.addChild(QTreeWidgetItem(["Total distance", "-m"]))
        items.append(item)

        for i in range(1, n_drones+1):
            item = QTreeWidgetItem(["UAV {}".format(i)])
            item.addChild(QTreeWidgetItem(["Battery", "-%"]))
            item.addChild(QTreeWidgetItem(["Position", "(-, -, -)"]))
            item.addChild(QTreeWidgetItem(["Time", "-s"]))
            item.addChild(QTreeWidgetItem(["Distance", "-m"]))
            item.addChild(QTreeWidgetItem(["Speed", "-m/s"]))
            items.append(item)

        self.drone_list.insertTopLevelItems(0, items)
    
        self.drone_dock = QDockWidget('UAVs', self)
        self.drone_dock.setWidget(self.drone_list)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.drone_dock)

        geometry = self.screen().availableGeometry()
        self.drone_list.setColumnWidth(0, geometry.width() * 0.12)
        self.drone_dock.setMinimumWidth(geometry.width() * 0.2)

    def _createMap(self):
        self.trj_map = MapWindow(self)
        self.setCentralWidget(self.trj_map)

    # ------------------------
    # Actions
    # ------------------------

    def _createActions(self):
        self.show_region_limits = QAction("&Regions to inspect", self, checkable=True)
        self.show_region_limits.setChecked(False)
        self.show_region_limits.triggered.connect(self.regionLimits)
        self.show_inspected_region = QAction("&Inspected regions", self, checkable=True)
        self.show_inspected_region.setChecked(False)
        self.show_inspected_region.triggered.connect(self.inspectedRegion)
        self.show_plan = QAction("&Plan", self, checkable=True)
        self.show_plan.setChecked(False)
        self.show_plan.triggered.connect(self.plan)
        self.show_plan_left = QAction("&Plan left", self, checkable=True)
        self.show_plan_left.setChecked(True)
        self.show_plan_left.triggered.connect(self.planLeft)
        self.show_real_trj = QAction("&Real trajectories", self, checkable=True)
        self.show_real_trj.setChecked(True)
        self.show_real_trj.triggered.connect(self.realTrajectories)

        # Exit action
        self.exit_action = QAction("&Exit", self)
        self.exit_action.setShortcut(QKeySequence.Quit)
        self.exit_action.triggered.connect(self.close)

    def inspectedRegion(self):
        self.trj_map.map.showInspectedSignal.emit(self.show_inspected_region.isChecked())

    def regionLimits(self):
        self.trj_map.map.showRegionSignal.emit(self.show_region_limits.isChecked())

    def realTrajectories(self):
        self.trj_map.map.showRealSignal.emit(self.show_real_trj.isChecked())

    def plan(self):
        self.trj_map.map.showPlanSignal.emit(self.show_plan.isChecked())

    def planLeft(self):
        self.trj_map.map.showPlanLeftSignal.emit(self.show_plan_left.isChecked())

    # ------------------------
    # ROS communication
    # ------------------------

    def subscribe(self):
        metrics_sub = rospy.Subscriber("performance_metrics", Metrics, self.metrics_callback)

    def metrics_callback(self, msg):
        self.metricsSignal.emit(msg)

    def change_metrics(self, msg):
        droneID = msg.identifier.natural
        battery = "{}%".format(msg.battery)
        distance = "{:.2f}m".format(msg.distance)
        speed = "{:.2f}m/s".format(msg.speed)
        position = "({:.2f}, {:.2f}, {:.2f})".format(msg.position.x, msg.position.y, msg.position.z)
        time = "{}s".format(msg.time)

        top_item = self.drone_list.topLevelItem(droneID)

        if (droneID == 0):
            top_item.child(0).setData(1, 0, time)
            top_item.child(1).setData(1, 0, distance)

        else:
            top_item.child(0).setData(1, 0, battery)
            top_item.child(1).setData(1, 0, position)
            top_item.child(2).setData(1, 0, time)
            top_item.child(3).setData(1, 0, distance)
            top_item.child(4).setData(1, 0, speed)
