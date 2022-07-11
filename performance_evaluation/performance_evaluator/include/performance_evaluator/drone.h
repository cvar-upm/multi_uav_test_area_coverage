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

#ifndef METRICS_DRONE_H
#define METRICS_DRONE_H

#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/BatteryState.h"

using namespace std;

class Drone {
public:
    Drone(int id);
    ~Drone(){};

private:
    int id;
    
    int battery = 100;
    double velocity = 0;
    double distance = 0;

    int time = 0;
    int startTime = 0;
    int endTime = -1;

    vector<double> position;

public: 
    int getId() {return id;}

    int getBattery() {return battery;}
    double getVelocity() {return velocity;}
    double getDistance() {return distance;}

    int getTime() {return time;}
    int getStartTime() {return startTime;}
    int getEndTime() {return endTime;}

    vector<double> getPosition() {return position;}

public:
    void setBattery(int battery) {this->battery = battery;}

    void setTime(int time) {this->time = time;}
    void setStartTime(int start) {startTime = start;}
    void setEndTime(int end) {endTime = end;}

public:
    void positionCallBack(const geometry_msgs::Pose &msg);
    void speedCallBack(const geometry_msgs::Twist &msg);
    void batteryCallBack(const sensor_msgs::BatteryState &msg);
};

#endif