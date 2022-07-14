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

#include "execution_monitor/monitor_info/global_drone.h"

GlobalDrone::GlobalDrone(int id) : MonitorData(id) {
    waypoints = vector<mutac_msgs::LabeledPoint>();
    time_last_msg = ros::Time::now();
    last_label.natural = mutac_msgs::Label::POSITIONING_LABEL;
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

void GlobalDrone::reset() {
    MonitorData::reset();
    waypoints = vector<mutac_msgs::LabeledPoint>();
    time_last_msg = ros::Time::now();
    last_label.natural = mutac_msgs::Label::POSITIONING_LABEL;
}
