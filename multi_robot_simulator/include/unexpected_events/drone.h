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

#ifndef DRONE_H
#define DRONE_H

#include <vector>
#include "Eigen/Dense"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/BatteryState.h"

#include "polynomial_trajectories/minimum_snap_trajectories.h"
#include "polynomial_trajectories/polynomial_trajectories_common.h"
#include "polynomial_trajectories/polynomial_trajectory.h"
#include "polynomial_trajectories/polynomial_trajectory_settings.h"
#include "quadrotor_common/trajectory_point.h"

enum class MotionState {
    NOT_STARTED,
    FOLLOWING_PLAN,
    FALLING_DOWN_FAILURE,
    GOING_HOMEBASE,
    STOPPED,
    RECTILINEAR_MOTION,
    SLOW_MOTION,
    FINISHED
};

enum class CameraState {
    CAMERA_OK,
    CAMERA_FAILURE,
    NO_CAMERA
};

using namespace polynomial_trajectories;

class Drone {
public:
    Drone(int id, std::vector<double> init_pos);
    Drone(const Drone &other);
    ~Drone(){};

private:
    int id;
    double battery;
    std::vector<double> position;

    int stopped_time;
    int event_stop_time;
    ros::Time start_stop;

    std::pair<MotionState, CameraState> state;
    std::pair<MotionState, CameraState> prev_state;

    bool trj_changed;
    std::vector<std::vector<double>> trj_points;
    std::shared_ptr<PolynomialTrajectory> trajectory;
    std::shared_ptr<PolynomialTrajectory> event_trajectory;
    
    double trj_velocity = 10;
    double trj_thrust = 10;

    Eigen::Vector3d acceleration;
    Eigen::Vector3d velocity;

    ros::Time timer;

    ros::NodeHandle nh;
    ros::Publisher battery_pub;

public: 
    int getId() {return id;}
    double getBattery() {return battery;}
    std::vector<double> getPosition() {return position;}

    int getStoppedTime() {return stopped_time;}
    int getEventStopTime() {return event_stop_time;}
    ros::Time getStartStop() {return start_stop;}

    bool getTrjChanged() {return trj_changed;}
    std::pair<MotionState, CameraState> getState() {return state;}
    std::pair<MotionState, CameraState> getPrevState() {return prev_state;}
    MotionState getMotionState() {return state.first;}
    CameraState getCameraState() {return state.second;}

    std::vector<std::vector<double>> getTrjPoints() {return trj_points;}
    std::shared_ptr<PolynomialTrajectory> getTrajectory() {return trajectory;}
    std::shared_ptr<PolynomialTrajectory> getEventTrajectory() {return event_trajectory;}

    double getTrjVelocity() {return trj_velocity;}
    double getTrjThrust() {return trj_thrust;}

    Eigen::Vector3d getAcceleration() {return acceleration;}
    Eigen::Vector3d getVelocity() {return velocity;}

    ros::Time getTimer() {return timer;}

public:
    void setBattery(double battery) {this->battery = battery;}
    void setPosition(std::vector<double> pos) {position[0] = pos[0]; position[1] = pos[1]; position[2] = pos[2];}

    void setStoppedTime(int stopped_time) {this->stopped_time = stopped_time;}
    void setEventStopTime(int event_stop_time) {this->event_stop_time = event_stop_time;}
    void setStartStop(ros::Time start_stop) {this->start_stop = start_stop;}

    void setTrjChanged(bool trj_changed) {this->trj_changed = trj_changed;}
    void setState(std::pair<MotionState, CameraState> state) {this->state = state;}
    void setPrevState(std::pair<MotionState, CameraState> prev_state) {this->prev_state = prev_state;}
    void setMotionState(MotionState motion_state) {state.first = motion_state;}
    void setCameraState(CameraState camera_state) {state.second = camera_state;}

    void setTrajectory(std::shared_ptr<PolynomialTrajectory> trajectory){this->trajectory = trajectory;}
    void setEventTrajectory(std::shared_ptr<PolynomialTrajectory> event_trajectory){this->event_trajectory = event_trajectory;}
    
    void setTrjVelocity(double trj_velocity) {this->trj_velocity = trj_velocity;}
    void setTrjThrust(double trj_thrust) {this->trj_thrust = trj_thrust;}

    void setAcceleration(Eigen::Vector3d acceleration) {this->acceleration = acceleration;}
    void setVelocity(Eigen::Vector3d velocity) {this->velocity = velocity;}

    void setTimer(ros::Time timer) {this->timer = timer;}

public:
    std::vector<double> getNextPoint(ros::Time time);
    void generateTrj(std::vector<Eigen::Vector3d> points, bool event);
    
    void addPoint(std::vector<double> point){trj_points.push_back(point);}
    void clearTrjPoints(){trj_points.clear();}

    void batteryDischarge(double percentage);

private:
    bool changeTrajectory();
    double calculateSegmentTime(Eigen::Vector3d p1, Eigen::Vector3d p2);
};

#endif