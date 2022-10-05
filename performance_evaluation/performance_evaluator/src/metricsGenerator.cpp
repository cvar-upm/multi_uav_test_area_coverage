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

#include "performance_evaluator/metricsGenerator.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "performance_evaluator");
    MetricsGenerator generator = MetricsGenerator();

    generator.start();

    return 0;
}

MetricsGenerator::MetricsGenerator() {
    nDrones = 0;

    if(!nh.getParam("/mutac/n_drones", nDrones)) {
        cerr << "\033[31;1mERROR: Missing parameter 'n_drones'.\033[0m" << endl;
        exit(1);
    }
    
    for (int i = 0; i < nDrones; i++) {
        shared_ptr<Drone> drone = make_shared<Drone>(i+1);
        drones.push_back(drone);
        pose_subs.push_back(nh.subscribe("drone" + to_string(i+1) + "/drone_pose", 1, &Drone::positionCallBack, drone.get()));
        speed_subs.push_back(nh.subscribe("drone" + to_string(i+1) + "/drone_twist", 1, &Drone::speedCallBack, drone.get()));
        battery_subs.push_back(nh.subscribe("drone" + to_string(i+1) + "/battery_state", 1, &Drone::batteryCallBack, drone.get()));
    }

    metrics_pub = nh.advertise<mutac_msgs::Metrics>("performance_metrics", 100);
    state_sub = nh.subscribe("drone_events", 100, &MetricsGenerator::stateCallBack, this);
    //trj_sub = nh.subscribe("planned_paths", 100, &MetricsGenerator::trajectoryCallBack, this);real_planned_paths
    trj_sub = nh.subscribe("real_planned_paths", 100, &MetricsGenerator::trajectoryCallBack, this);
}

// Sends messages with the global metrics and the drones metrics with a frequency of 10Hz
void MetricsGenerator::start() {
    ros::Rate rate(5);
    while (ros::ok() && drones.size() > 0) {
        for (int i = 0; i <= this->drones.size(); i++) {
            mutac_msgs::Metrics msg = generateMsg(i);
            metrics_pub.publish(msg);
        }
        
        ros::spinOnce();
        rate.sleep();
  }
}

/* Generates the metrics of an especified element.
    id == 0 -> global metrics
    id > 0 -> drone metrics
*/
mutac_msgs::Metrics MetricsGenerator::generateMsg(int id) {
    mutac_msgs::Metrics msg = mutac_msgs::Metrics();
    msg.identifier.natural = id;

    if (id == 0) {
        msg.distance = getGlobalDistance();
        msg.time = getGlobalTime();
    }
    else {
        shared_ptr<Drone> drone = this->drones[id-1];
        
        msg.battery = drone->getBattery();
        msg.speed = drone->getVelocity();
        msg.distance = drone->getDistance();
        msg.time = drone->getTime();

        msg.position.x = drone->getPosition()[0];
        msg.position.y = drone->getPosition()[1];
        msg.position.z = drone->getPosition()[2];
    }

    return msg;
}

// Calculates the total distance flied by the drones
int MetricsGenerator::getGlobalDistance() {
    int dist = 0;
    for (int i = 0; i < this->drones.size(); i++) {
        dist += this->drones[i]->getDistance();
    }
    return dist;
}

// Calculates the time that has passed since the start of the first drone
int MetricsGenerator::getGlobalTime() {
    int time;
    int start = INT_MAX;
    int end = INT_MIN;
    int started = false;
    int ended = true;

    for (int i = 0; i < drones.size(); i++) {
        int start_time = drones[i]->getStartTime();
        int end_time = drones[i]->getEndTime();

        start = (start_time > 0 && start_time < start) ? start_time:start;
        end = (end_time >= 0 && end_time > end) ? end_time:end;
        
        started = started || start_time != 0;
        ended = ended && end_time >= 0;
    }

    if (!started) time = 0;
    else if (!ended) time = ros::Time::now().toSec() - start;
    else time = end - start;

    return time;
}

// Changes the state of the drone to synchronize times
void MetricsGenerator::stateCallBack(const mutac_msgs::State &msg) {
    shared_ptr<Drone> drone = drones[msg.identifier.natural];
    int start = drone->getStartTime();

    if(msg.state == msg.LOST || msg.state == msg.LANDED) {
        if(msg.state == msg.LOST) {
            drone->setStartLost(ros::Time::now().toSec());
        }
        drone->setEndTime(ros::Time::now().toSec());
        drone->setTime(ros::Time::now().toSec() -  start - drone->getLostTime());
    } else if(msg.state == msg.RECOVERED) {
        drone->setLostTime(drone->getLostTime() + ros::Time::now().toSec() -  drone->getStartLost());
        drone->setEndTime(-1);
    }
}

void MetricsGenerator::trajectoryCallBack(const mutac_msgs::Plan &msg) {
    for (size_t i = 0; i < msg.paths.size(); i++) {
        if (drones[msg.paths[i].identifier.natural]->getStartTime() == 0)
            drones[msg.paths[i].identifier.natural]->setStartTime(ros::Time::now().toSec());
    }
}
