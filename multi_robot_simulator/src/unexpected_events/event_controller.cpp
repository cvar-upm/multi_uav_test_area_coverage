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

#include "unexpected_events/event_controller.h"

EventController::EventController(std::shared_ptr<Drone> drone, std::vector<double> homebase, double floorZ) :
    drone(drone), homebase(homebase), floorZ(floorZ) {
        nh = ros::NodeHandle("");
        alarm_pub = nh.advertise<mutac_msgs::Alarm>("drone_alarm", 100);
        //battery_pub = nh.advertise<sensor_msgs::BatteryState>("drone" + to_string(drone->getId()+1) + "/battery_state", 100);
}

EventController::EventController(const EventController &other) :
    drone(other.drone), homebase(other.homebase), floorZ(other.floorZ), timer(other.timer), nh(other.nh), //battery_pub(other.battery_pub),
    alarm_pub(other.alarm_pub) {}

void EventController::startEvent(Event event) {
    double instantBattery;
    switch (event.getType()) {
    case EventType::BLIND_CAMERA:
        std::cout << "Drone: " << drone->getId() << " Event: BLIND_CAMERA" << std::endl;
        blindCamera();
        break;
    case EventType::GO_HOMEBASE:
        std::cout << "Drone: " << drone->getId() << " Event: GO_HOMEBASE" << std::endl;
        goHomeBase();
        break;
    case EventType::BATTERY_DISCHARGE:
        std::cout << "Drone: " << drone->getId() << " Event: BATTERY_DISCHARGE" << std::endl;
        drone->batteryDischarge(event.getParam());
        instantBattery = drone->getBattery();
        if(instantBattery <= 0) {
            fallDown();
        }
        break;
    case EventType::FALL_DOWN:
        std::cout << "Drone: " << drone->getId() << " Event: FALL_DOWN" << std::endl;
        fallDown();
        break;
    case EventType::STOPPED:
        std::cout << "Drone: " << drone->getId() << " Event: STOPPED" << std::endl;
        stop(event.getParam());
        break;
    case EventType::RECTILINEAR_MOTION:
        std::cout << "Drone: " << drone->getId() << " Event: RECTILINEAR_MOTION" << std::endl;
        rectilinearMotion();
        break;
    case EventType::SLOW_MOTION:
        std::cout << "Drone: " << drone->getId() << " Event: SLOW_MOTION" << std::endl;
        slowMotion();
        break;
    }
}

void EventController::timerCallback(const ros::TimerEvent&) {
    drone->setState(drone->getPrevState());
}

/* ------
Events
------ */

void EventController::blindCamera() {
    drone->setCameraState(CameraState::CAMERA_FAILURE);
    mutac_msgs::Alarm msg = mutac_msgs::Alarm();
    msg.identifier.natural = drone->getId();
    msg.camera = false;
    alarm_pub.publish(msg);
}

void EventController::goHomeBase() {
    drone->setMotionState(MotionState::GOING_HOMEBASE);
    
    Eigen::Vector3d start {drone->getPosition()[0], drone->getPosition()[1], drone->getPosition()[2]};
    Eigen::Vector3d end {homebase[0], homebase[1], homebase[2]};
    std::vector<Eigen::Vector3d> points {start, end};

    drone->generateTrj(points, true);

    drone->setTrjChanged(true);
}
/*
void EventController::batteryDischarge(double percentage) {
    sensor_msgs::BatteryState msg = sensor_msgs::BatteryState();
    msg.percentage = percentage;//drone->getBattery() - percentage;
    
    if (msg.percentage <= 0) {
        msg.percentage = 0;
        fallDown();
    }

    drone->setBattery(msg.percentage);
    battery_pub.publish(msg);
}*/

void EventController::fallDown() {
    drone->setMotionState(MotionState::FALLING_DOWN_FAILURE);
    
    Eigen::Vector3d start(drone->getPosition()[0], drone->getPosition()[1], drone->getPosition()[2]);
    Eigen::Vector3d end(drone->getPosition()[0], drone->getPosition()[1], floorZ);
    std::vector<Eigen::Vector3d> points {start, end};

    drone->generateTrj(points, true);
    drone->setTrjChanged(true);
}

void EventController::stop(int secs) {
    drone->setPrevState(drone->getState());
    drone->setMotionState(MotionState::STOPPED);
    
    drone->setStoppedTime(drone->getStoppedTime() + secs);
    drone->setEventStopTime(secs);
    drone->setStartStop(ros::Time::now());

    drone->setVelocity(Eigen::Vector3d(0, 0, 0));

    timer = nh.createTimer(ros::Duration(secs), &EventController::timerCallback, this, true);
}

void EventController::rectilinearMotion() {
    drone->setMotionState(MotionState::RECTILINEAR_MOTION);
    
    Eigen::Vector3d start(drone->getPosition()[0], drone->getPosition()[1], drone->getPosition()[2]);
    Eigen::Vector3d end(drone->getPosition()[0] + 700, drone->getPosition()[1], drone->getPosition()[2]);
    std::vector<Eigen::Vector3d> points {start, end};

    drone->generateTrj(points, true);
    drone->setTrjChanged(true);
}

void EventController::slowMotion() {
    drone->setMotionState(MotionState::SLOW_MOTION);
    Eigen::Vector3d start(drone->getPosition()[0], drone->getPosition()[1], drone->getPosition()[2]);

    Eigen::Vector3d end(drone->getTrjPoints()[0][0], drone->getTrjPoints()[0][1], drone->getTrjPoints()[0][2]);
    std::vector<Eigen::Vector3d> points {start, end};

    drone->setTrjVelocity(5);
    drone->setTrjThrust(10);
    
    drone->generateTrj(points, false);
    drone->setTrjChanged(true);
}
