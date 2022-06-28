#include "execution_monitor/monitor_info/global_drone.h"

GlobalDrone::GlobalDrone(int id) : MonitorData(id) {
    waypoints = vector<mutac_msgs::LabeledPoint>();
    time_last_msg = ros::Time::now();
    last_label.natural = mutac_msgs::Label::COVERING_LABEL;
}

// Cehcks if there is still connection with the drone
void GlobalDrone::checkConnection(double time_threshold) {
    double time_pose = (ros::Time::now() - time_last_msg).toSec();
    if (state != State::NOT_STARTED && time_pose > time_threshold) state = State::LOST;
}

/* ----------------
ROS Communication
---------------- */ 

void GlobalDrone::positionCallBack(const geometry_msgs::Pose &msg) {
    MonitorData::positionCallBack(msg);
    time_last_msg = ros::Time::now();
}

/* ---------
Waypoints
--------- */

void GlobalDrone::setWaypoints(vector<mutac_msgs::LabeledPoint> waypoints) {
    waypoints.erase(waypoints.begin());
    this->waypoints = waypoints; 
}

// Advances the waypoints list in one
void GlobalDrone::advanceWP() {
    last_label = waypoints[0].label;
    waypoints.erase(waypoints.begin());
}
