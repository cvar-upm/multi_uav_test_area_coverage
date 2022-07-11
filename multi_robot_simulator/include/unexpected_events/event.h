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