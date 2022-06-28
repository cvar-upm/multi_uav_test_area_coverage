#ifndef MONITOR_DATA_H
#define MONITOR_DATA_H

#include <vector>
#include "geometry_msgs/Pose.h"

using namespace std;

enum class State {
    NOT_STARTED,
    ON_MISSION,
    MISSION_FINISHED,
    GOING_HOME,
    LANDED,
    LOST
};

class MonitorData {
public:
    MonitorData(int id);
    MonitorData(const MonitorData &other);
    ~MonitorData(){}

protected:
    int id;
    State state;
    
    vector<double> position;
    vector<double> pos_in_trj;

public:
    int getId() {return id;}
    State getState() {return state;}

    vector<double> getPosition() {return position;}
    vector<double> getPosInTrj() {return pos_in_trj;}

public:
    void setState(State state) {this->state = state;}
    void setPosInTrj(vector<double> pos) {pos_in_trj = pos;}

public:
    void positionCallBack(const geometry_msgs::Pose &msg);
};

#endif