#ifndef EVENT_CONTROLLER_H
#define EVENT_CONTROLLER_H

#include "unexpected_events/drone.h"
#include "unexpected_events/event.h"

#include <vector>
#include "Eigen/Dense"

// ROS
#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
#include "mutac_msgs/Alarm.h"

// Flightlib
#include "polynomial_trajectories/minimum_snap_trajectories.h"
#include "polynomial_trajectories/polynomial_trajectories_common.h"
#include "polynomial_trajectories/polynomial_trajectory.h"
#include "polynomial_trajectories/polynomial_trajectory_settings.h"
#include "quadrotor_common/trajectory_point.h"

using namespace polynomial_trajectories;

class EventController {
public:
    EventController(std::shared_ptr<Drone> drone, std::vector<double> homebase, double floorZ);
    EventController(const EventController &other);
    ~EventController(){};

private:
    std::shared_ptr<Drone> drone;
    std::vector<double> homebase;
    double floorZ;

    ros::Timer timer;
    
    ros::NodeHandle nh;
    ros::Publisher battery_pub;
    ros::Publisher alarm_pub;

public:
    void startEvent(Event event);
    
private:
    void blindCamera();
    void goHomeBase();
    void batteryDischarge(int percentage);
    void fallDown();
    void stop(int secs);
    void rectilinearMotion();
    void slowMotion();

    void timerCallback(const ros::TimerEvent&);
};

#endif