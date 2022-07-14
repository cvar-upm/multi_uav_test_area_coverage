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
    map<int, shared_ptr<GlobalDrone>> lost_drones;

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