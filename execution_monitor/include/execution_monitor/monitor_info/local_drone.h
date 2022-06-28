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
};

#endif