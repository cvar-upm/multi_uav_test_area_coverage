#ifndef EVENT_H
#define EVENT_H

#include <string>
#include <vector>

using namespace std;

enum class EventType {
    BLIND_CAMERA,
    GO_HOMEBASE,
    BATTERY_DISCHARGE,
    FALL_DOWN,
    STOPPED,
    RECTILINEAR_MOTION,
    SLOW_MOTION,
    WRONG_EVENT
};

class Event {
public:
    Event(int droneID, int time, string type, int param);
    Event(int droneID, int time, string type);
    Event(int droneID, EventType type, int param);
    Event(){};
    ~Event(){};

private:
    int droneID;
    int param;
    int time;
    EventType type;

public:
    int getDroneID() {return droneID;}
    int getParam() {return param;}
    int getTime() {return time;}
    EventType getType() {return type;}

    void setType(EventType type) {this->type = type;}
    void setParam(int param) {this->param = param;}
    void setTime(int time) {this->time = time;}
    void setDroneID(int droneID) {this->droneID = droneID;}

    bool operator < (const Event& event) const {
        return (time < event.time);
    }

private:
    EventType eventType(string eventName);
};

#endif