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
    void reset();
};

#endif