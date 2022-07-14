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

#include "execution_monitor/execution_monitors/execution_monitor.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, ros::this_node::getName());    
    ros::NodeHandle pnh = ros::NodeHandle("~");
    
    int id;
    pnh.getParam("drone_id", id);

    ExecutionMonitor monitor = ExecutionMonitor(id);
    monitor.start();

    return 0;
}

ExecutionMonitor::ExecutionMonitor(int id) {
    this->id = id-1;
    drone = make_shared<LocalDrone>(id-1);

    // ROS Communication
    nh = ros::NodeHandle("");
    
    event_pub = nh.advertise<mutac_msgs::State>("/mutac/drone_events", 100);
    covered_pub = nh.advertise<mutac_msgs::Identifier>("/mutac/covered_points", 100);

    pose_sub = nh.subscribe("drone_pose", 100, &LocalDrone::positionCallBack, dynamic_cast<MonitorData*>(drone.get()));
    battery_sub = nh.subscribe("battery_state", 100, &LocalDrone::batteryCallBack, drone.get());

    trj_sub = nh.subscribe("/mutac/planned_paths", 100, &ExecutionMonitor::trajectoryCallBack, this);
    alarm_sub = nh.subscribe("/mutac/drone_alarm", 100, &ExecutionMonitor::cameraCallBack, this);
}

/* Keeps checking the state of the drone.
The execution monitor sends a message to notify the global monitor if the drone:
    - passed through a waypoint
    - finished the inspection
    - got lost
    - landed
If the drone got lost or landed, the execution monitor finishes its execution. */
void ExecutionMonitor::start() {
    ros::Rate rate(3);
    bool stop = false;

    while (ros::ok() && !stop) {
        int eventID = drone->checkDrone(dist_trj, dist_WP);
        
        // Sends notification if drone finished inspection or got lost
        if (eventID == 0 || eventID == 1) {
            vector<double> pos = drone->hasDeviated() ? drone->getPosInTrj() : drone->getPosition();
            mutac_msgs::State msg = mutac_msgs::State();
            msg.identifier.natural = id;
            msg.state = eventID;
            msg.position.x = pos[0];
            msg.position.y = pos[1];
            msg.position.z = pos[2];
            event_pub.publish(msg);

            //if(eventID == 1) stop = true;
        }

        // Sends notification if drone landed
        else if (eventID == 2) {
            mutac_msgs::State msg = mutac_msgs::State();
            msg.identifier.natural = id;
            msg.state = eventID;
            event_pub.publish(msg);
            
            stop = true;
        }

        // Sends notification if waypoint checked
        else if (eventID == 3) {
            mutac_msgs::Identifier msg = mutac_msgs::Identifier();
            msg.natural = id;
            covered_pub.publish(msg);
        }

        if (stop) ros::Duration(5.0).sleep();
        ros::spinOnce();
        rate.sleep();
    }
}

/* Looks for the path of an specific drone.
Returns the index of the path or -1 if it wasn't found.*/
int ExecutionMonitor::findPath(vector<mutac_msgs::LabeledPath> plans, int droneID) {
    for (size_t i = 0; i < plans.size(); i++) {
        if (plans[i].identifier.natural == droneID) return i;
    }
    return -1;
}

/* ----------------
ROS Communication
---------------- */ 
void ExecutionMonitor::trajectoryCallBack(const mutac_msgs::Plan &msg) {
    vector<mutac_msgs::LabeledPath> plans = msg.paths;
    vector<vector<double>> wps;
    vector<vector<double>> insp_wps;

    int lastLabel = mutac_msgs::Label::POSITIONING_LABEL;
    int index = findPath(plans, id);

    if (index < 0) return;
    vector<mutac_msgs::LabeledPoint> path = plans[index].points;

    for (size_t i = 0; i < path.size(); i++) {
        if (lastLabel == mutac_msgs::Label::COVERING_LABEL && 
            path[i].label.natural == mutac_msgs::Label::POSITIONING_LABEL) {
            insp_wps = wps;
        }

        wps.push_back(vector<double> {path[i].point.x, path[i].point.y, path[i].point.z});
        lastLabel = path[i].label.natural;
    }

    drone->setWaypoints(wps);
    drone->setInspectionWPS(insp_wps);
    drone->setState(State::ON_MISSION);
}

void ExecutionMonitor::cameraCallBack(const mutac_msgs::Alarm &msg) {
    //std::cout << "UNO" << std::endl;
    if (msg.identifier.natural == id) {
        //std::cout << "DOS" << std::endl;
        switch(msg.alarm) {
            case mutac_msgs::Alarm::CAMERA_FAILURE:
                //std::cout << "TRES" << std::endl;
                drone->setCamera(false);
                break;
            case mutac_msgs::Alarm::DRONE_RECOVERED:
                //std::cout << "CUATRO" << std::endl;
                drone->reset();
                mutac_msgs::State msg = mutac_msgs::State();
                msg.identifier.natural = id;
                msg.state = mutac_msgs::State::RECOVERED;
                event_pub.publish(msg);
                break;
        }
    }
}

