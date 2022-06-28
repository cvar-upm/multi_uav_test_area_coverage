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