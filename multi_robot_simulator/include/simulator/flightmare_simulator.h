#ifndef MULTI_ROBOT_SIMULATOR_H
#define MULTI_ROBOT_SIMULATOR_H

#include "unexpected_events/drone.h"
#include "unexpected_events/event.h"
#include "unexpected_events/event_controller.h"
#include "unexpected_events/event_generator.h"

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include "Eigen/Dense"

// ROS
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "mutac_msgs/Plan.h"
#include "mutac_msgs/Alarm.h"
#include "mutac_msgs/State.h"
#include "mutac_msgs/LabeledPoint.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

// Flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

#include "polynomial_trajectories/minimum_snap_trajectories.h"
#include "polynomial_trajectories/polynomial_trajectories_common.h"
#include "polynomial_trajectories/polynomial_trajectory.h"
#include "polynomial_trajectories/polynomial_trajectory_settings.h"
#include "quadrotor_common/trajectory_point.h"

using namespace flightlib;

class Simulator {
public:
    Simulator();
    ~Simulator(){};
    void start();

private:
    int n_drones;
    std::vector<std::vector<double>> homebases;
    double floor;
    bool all_cameras;

    std::vector<std::shared_ptr<Drone>> drones;
    std::vector<EventController> event_controllers;
    EventGenerator event_gen;

    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber path_sub;

    std::vector<ros::Publisher> pose_pubs;
    std::vector<ros::Publisher> speed_pubs;

    bool unity_ready;
    std::vector<std::shared_ptr<Quadrotor>> quad_ptrs;
    std::vector<QuadState> quad_states;
    std::shared_ptr<UnityBridge> unity_bridge_ptr;
    std::map<int, image_transport::Publisher> rgb_pubs;
    std::map<int, std::shared_ptr<RGBCamera>> rgb_cameras;

private:
    void connectROS();
    void configDrones();
    void configUnity(SceneID sceneId);

    void pathCallback(const mutac_msgs::Plan &msg);

    void restartTimer();
};

#endif