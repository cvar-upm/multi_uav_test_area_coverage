#include "performance_evaluator/drone.h"

Drone::Drone(int id) {
    this->id = id;
    position = vector<double>(3);
}

void Drone::positionCallBack(const geometry_msgs::Pose &msg) {
    if (startTime > 0 && endTime < 0) {
        distance += sqrt(pow(msg.position.x - position[0], 2) + pow(msg.position.y - position[1], 2) +
                        pow(msg.position.z - position[2], 2));
        time = ros::Time::now().toSec() - startTime;
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