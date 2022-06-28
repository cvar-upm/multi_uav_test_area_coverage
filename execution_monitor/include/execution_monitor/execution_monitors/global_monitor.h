#ifndef GLOBAL_MONITOR_H
#define GLOBAL_MONITOR_H

#include <vector>
#include <map>

#include "ros/ros.h"
#include "mutac_msgs/State.h"
#include "mutac_msgs/UpdatePlan.h"
#include "mutac_msgs/Plan.h"
#include "mutac_msgs/LabeledPath.h"
#include "mutac_msgs/LabeledPoint.h"

#include "execution_monitor/monitor_info/global_drone.h"

using namespace std;

class GlobalMonitor {
public:
    GlobalMonitor();
    ~GlobalMonitor(){};
    void start();

private:
    int nDrones;
    map<int, shared_ptr<GlobalDrone>> drones;

    double last_msg_time = 5;

    // ROS
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    vector<ros::Subscriber> pose_subs;
    ros::Subscriber covered_sub;
    ros::Subscriber events_sub;
    ros::Subscriber trj_sub;

    ros::ServiceClient replan_client;

private:
    void askReplan();
    vector<mutac_msgs::LabeledPath> generatePlanPaths();

    void eventCallBack(const mutac_msgs::State &msg);
    void waypointCallBack(const mutac_msgs::Identifier &msg);
    void trajectoryCallBack(const mutac_msgs::Plan &msg);
};

#endif