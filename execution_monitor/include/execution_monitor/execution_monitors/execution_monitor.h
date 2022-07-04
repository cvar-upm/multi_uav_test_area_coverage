#ifndef EXECUTION_MONITOR_H
#define EXECUTION_MONITOR_H

#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "mutac_msgs/Alarm.h"
#include "mutac_msgs/State.h"
#include "mutac_msgs/Plan.h"
#include "mutac_msgs/LabeledPath.h"
#include "mutac_msgs/LabeledPoint.h"

#include "execution_monitor/monitor_info/local_drone.h"

using namespace std;

class ExecutionMonitor {
public:
    ExecutionMonitor(int id);
    ~ExecutionMonitor(){};
    void start();

private:
    shared_ptr<LocalDrone> drone;
    int id;

    double dist_trj = 5;
    double dist_WP = 2;

    // ROS
    ros::NodeHandle nh;

    ros::Publisher event_pub;
    ros::Publisher covered_pub;
    
    ros::Subscriber trj_sub;
    ros::Subscriber alarm_sub;
    ros::Subscriber battery_sub;
    ros::Subscriber pose_sub;

private:
    void trajectoryCallBack(const mutac_msgs::Plan &msg);
    void cameraCallBack(const mutac_msgs::Alarm &msg);

    int findPath(vector<mutac_msgs::LabeledPath> plans, int droneID);
};

#endif