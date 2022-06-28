#ifndef EVENT_GENERATOR_H
#define EVENT_GENERATOR_H

#include "unexpected_events/event.h"

#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include "ros/ros.h"

using namespace std;

class EventGenerator {
public:
    EventGenerator(string path);
    EventGenerator(){};
    ~EventGenerator(){};

private:
    int start = -1;
    vector<Event> events;

public:
    int getStart() {return start;}
    void setStart(int start) {this->start = start;}
    
    vector<Event> genEvents();
};

#endif