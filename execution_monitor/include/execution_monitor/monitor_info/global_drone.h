#ifndef GLOBAL_DRONE_H
#define GLOBAL_DRONE_H

#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "mutac_msgs/LabeledPoint.h"
#include "mutac_msgs/Label.h"

#include "execution_monitor/monitor_info/monitor_data.h"

using namespace std;

class GlobalDrone : public MonitorData {
public:
    GlobalDrone(int id);
    ~GlobalDrone(){}

private:
    ros::Time time_last_msg;
    mutac_msgs::Label last_label;
    State last_state;
    vector<mutac_msgs::LabeledPoint> waypoints;
    size_t index;

public:
    mutac_msgs::Label getLastLabel() {return last_label;}
    State getLastState() {return last_state;}
    vector<mutac_msgs::LabeledPoint> getWaypoints() {return waypoints;}
    size_t getIndex(){return index;}
    
    void setWaypoints(vector<mutac_msgs::LabeledPoint> waypoints);
    void setIndex(size_t index) {this->index = index;}

public:
    void advanceWP();
    void saveState() {last_state = state;}
    void checkConnection(double time_threshold);
    
    void positionCallBack(const geometry_msgs::Pose &msg);
};

#endif