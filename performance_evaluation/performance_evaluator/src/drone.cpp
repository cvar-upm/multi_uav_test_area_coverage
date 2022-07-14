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

#include "performance_evaluator/drone.h"

Drone::Drone(int id) {
    this->id = id;
    position = vector<double>(3);
}

void Drone::positionCallBack(const geometry_msgs::Pose &msg) {
    if (startTime > 0 && endTime < 0) {
        distance += sqrt(pow(msg.position.x - position[0], 2) + pow(msg.position.y - position[1], 2) +
                        pow(msg.position.z - position[2], 2));
        time = ros::Time::now().toSec() - startTime - lostTime;
    }

    position[0] = msg.position.x;
    position[1] = msg.position.y;
    position[2] = msg.position.z;
}

void Drone::speedCallBack(const geometry_msgs::Twist &msg) {
    velocity = sqrt(pow(msg.linear.x, 2) + pow(msg.linear.y, 2) + pow(msg.linear.z, 2));
}

void Drone::batteryCallBack(const sensor_msgs::BatteryState &msg) {
    battery = msg.percentage;
}