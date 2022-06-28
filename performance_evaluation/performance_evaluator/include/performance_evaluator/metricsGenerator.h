#ifndef METRICS_H
#define METRICS_H

#include <math.h>
#include "performance_evaluator/drone.h"

// ROS
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "mutac_msgs/Metrics.h"
#include "mutac_msgs/State.h"
#include "mutac_msgs/Plan.h"

using namespace std;

class MetricsGenerator {
public:
    MetricsGenerator();
    ~MetricsGenerator(){};
    void start();

private:
    int nDrones;
    ros::NodeHandle nh;
    vector<shared_ptr<Drone>> drones;

    // ROS communication
    ros::Publisher metrics_pub;
    vector<ros::Subscriber> pose_subs;
    vector<ros::Subscriber> speed_subs;
    vector<ros::Subscriber> battery_subs;
    ros::Subscriber trj_sub;
    ros::Subscriber state_sub;

private:
    mutac_msgs::Metrics generateMsg(int id);
    int getGlobalDistance();
    int getGlobalTime();

    void stateCallBack(const mutac_msgs::State &msg);
    void trajectoryCallBack(const mutac_msgs::Plan &msg);
};

#endif
