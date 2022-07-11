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
