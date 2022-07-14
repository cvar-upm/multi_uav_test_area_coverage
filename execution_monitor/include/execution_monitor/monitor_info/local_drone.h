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

#ifndef LOCAL_DRONE_H
#define LOCAL_DRONE_H

#include <vector>

#include "geometry_msgs/Pose.h"
#include "sensor_msgs/BatteryState.h"

#include "execution_monitor/monitor_info/monitor_data.h"

using namespace std;

class LocalDrone : public MonitorData {
public:
    LocalDrone(int id);
    ~LocalDrone(){}

private:
    int battery;
    bool camera;
    bool deviated;

    vector<vector<double>> waypoints;
    vector<vector<double>> inspection_wps;

    double prev_distance;
    vector<double> lastWP;

public:
    int getBattery() {return battery;}
    bool cameraOn() {return camera;}
    bool hasDeviated() {return deviated;}

public:
    void setBattery(int battery) {this->battery = battery;}
    void setCamera(bool camera) {this->camera = camera;}

    void setInspectionWPS(vector<vector<double>> inspection_wps);
    void setWaypoints(vector<vector<double>> waypoints);

private:
    double distance(vector<double> p1, vector<double> p2);
    double distanceToTrj(vector<double> pos, vector<double> p1, vector<double> p2);
    double vectorNorm(double x, double y, double z);
    vector<double> calculateProjection(vector<double> pos, vector<double> wp1, vector<double> wp2);

public:
    int checkDrone(double distTrj, double distWP);
    void batteryCallBack(const sensor_msgs::BatteryState &msg);
    void advanceWP();
    void reset();
};

#endif