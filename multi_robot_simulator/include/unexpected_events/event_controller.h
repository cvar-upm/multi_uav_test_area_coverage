/*!******************************************************************************
 * \authors   Elisa Martinez-Abad
 * \copyright Copyright (c) 2022 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

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
    //ros::Publisher battery_pub;
    //ros::Publisher alarm_pub;

public:
    void startEvent(Event event);
    
private:
    void blindCamera();
    void goHomeBase();
    //void batteryDischarge(double percentage);
    void fallDown();
    void stop(int secs);
    void rectilinearMotion();
    void slowMotion();

    void timerCallback(const ros::TimerEvent&);
};

#endif