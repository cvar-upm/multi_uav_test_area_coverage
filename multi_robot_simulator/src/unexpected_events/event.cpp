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
