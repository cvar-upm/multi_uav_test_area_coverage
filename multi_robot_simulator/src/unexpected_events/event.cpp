#include "unexpected_events/event.h"

Event::Event (int droneID, int time, std::string type, int param) :
    droneID(droneID), param(param), time(time), type(eventType(type)) {}

Event::Event(int droneID, int time, std::string type) :
    droneID(droneID), param(0), time(time), type(eventType(type)) {}

Event::Event(int droneID, EventType type, int param) :
    droneID(droneID), param(param), type(type) {}

EventType Event::eventType(std::string eventName) {
    if (eventName == "BLIND_CAMERA") 
        return EventType::BLIND_CAMERA;
    else if (eventName == "GO_HOMEBASE") 
        return EventType::GO_HOMEBASE;
    else if (eventName == "BATTERY_DISCHARGE") 
        return EventType::BATTERY_DISCHARGE;
    else if (eventName == "FALL_DOWN") 
        return EventType::FALL_DOWN;
    else if (eventName == "STOPPED") 
        return EventType::STOPPED;
    else if (eventName == "RECTILINEAR_MOTION") 
        return EventType::RECTILINEAR_MOTION;
    else if (eventName == "SLOW_MOTION") 
        return EventType::SLOW_MOTION;
        
    return EventType::WRONG_EVENT;
}
