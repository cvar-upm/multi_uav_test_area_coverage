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

#include "execution_monitor/execution_monitors/global_monitor.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "global_execution_monitor");
    GlobalMonitor monitor = GlobalMonitor();
    monitor.start();

    return 0;
}

GlobalMonitor::GlobalMonitor() {
    nh = ros::NodeHandle("");
    pnh = ros::NodeHandle("~");
    
    if (!nh.getParam("/mutac/n_drones", nDrones)) {
        cerr << "\033[31;1mERROR: Missing parameter 'n_drones'.\033[0m" << endl;
        exit(1);
    }
    
    for (int i = 0; i < nDrones; i++) {
        shared_ptr<GlobalDrone> drone = make_shared<GlobalDrone>(i);
        drones[i] = drone;

        pose_subs.push_back(nh.subscribe("drone" + to_string(i+1) + "/drone_pose", 1, &GlobalDrone::positionCallBack, drone.get()));
    }

    //trj_sub = nh.subscribe("planned_paths", 100, &GlobalMonitor::trajectoryCallBack, this);
    trj_sub = nh.subscribe("real_planned_paths", 100, &GlobalMonitor::trajectoryCallBack, this);
    events_sub = nh.subscribe("drone_events", 100, &GlobalMonitor::eventCallBack, this);
    covered_sub = nh.subscribe("covered_points", 100, &GlobalMonitor::waypointCallBack, this);

    replan_client = nh.serviceClient<mutac_msgs::UpdatePlan>("update_plan");
}

/* Keeps checking the state of the drones.
If the drone got lost or has finished the inspection mission, the monitor asks for a mission replan. */
void GlobalMonitor::start() {
    ros::Rate rate(3);

    while (ros::ok() && drones.size() > 0) {
        for(auto it = drones.cbegin(); it != drones.cend();) {
            bool erased = false;
            int droneID = (*it).first;
            shared_ptr<GlobalDrone> drone = (*it).second;
            
            State state = drone->getState();
            drone->checkConnection(last_msg_time);

            // Remove from list if drone died
            if (state == State::LOST) {
                cout << "GLOBAL MONITOR: Drone " << droneID + 1 << " lost." << endl;
                
                // Only ask for a replan if the drone got lost while on mission and there are drones left
                if (drone->getLastState() == State::ON_MISSION && drones.size() > 1)
                    askReplan();
                
                //pose_subs[droneID].shutdown();
                lost_drones[droneID] = drone;
                it = drones.erase(it);
                erased = true;
            }
            
            else if (state == State::LANDED) {
                cout << "GLOBAL MONITOR: Drone " << droneID + 1 << " landed." << endl;

                pose_subs[droneID].shutdown();
                it = drones.erase(it);
                erased = true;
            }

            else if (state == State::MISSION_FINISHED) {
                cout << "GLOBAL MONITOR: Drone " << droneID + 1 << " has finished." << endl;
                
                if (drones.size() > 1)
                    askReplan();
                    
                drone->setState(State::GOING_HOME);
            }

            if(!erased) it++;
        }
        if(drones.size() > 0) ros::spinOnce();
        rate.sleep();
    }
}

/* ------
Replan
------ */

void GlobalMonitor::askReplan() {
    mutac_msgs::UpdatePlan srv = mutac_msgs::UpdatePlan();
    srv.request.plan.paths = generatePlanPaths();

    replan_client.call(srv);
}

vector<mutac_msgs::LabeledPath> GlobalMonitor::generatePlanPaths() {
    vector<mutac_msgs::LabeledPath> paths(drones.size());

    for(auto it = drones.cbegin(); it != drones.cend();) {
        int droneID = (*it).first;
        shared_ptr<GlobalDrone> drone = (*it).second;
        State state = drone->getState();

        mutac_msgs::LabeledPoint point = mutac_msgs::LabeledPoint();
        vector<double> pos = (state != State::LOST) ? drone->getPosition() : drone->getPosInTrj();
        point.point.x = pos[0];
        point.point.y = pos[1];
        point.point.z = pos[2];

        mutac_msgs::LabeledPath path = mutac_msgs::LabeledPath();

        path.identifier.natural = state != State::LOST ? droneID : -1;
        path.points = drone->getWaypoints();

        point.label.natural = (path.points.size() <= 0 || path.points[0].label != drone->getLastLabel()) ? mutac_msgs::Label::POSITIONING_LABEL : path.points[0].label.natural;
        path.points.insert(path.points.begin(), point);

        paths[drone->getIndex()] = path;

        //std::cout << droneID << ":" << std::endl;
        //std::cout << path << std::endl;

        it++;
    }

    std::cout << paths[0] << std::endl << std::endl << paths[1] << std::endl;
    std::cout << "------------------------------" << std::endl;

    return paths;
}

/* ----------------
ROS Communication
---------------- */

void GlobalMonitor::trajectoryCallBack(const mutac_msgs::Plan &msg) {
    vector<mutac_msgs::LabeledPath> plans = msg.paths;

    for (size_t i = 0; i < plans.size(); i++) {        
        drones[plans[i].identifier.natural]->setWaypoints(plans[i].points);
        drones[plans[i].identifier.natural]->setState(State::ON_MISSION);
        drones[plans[i].identifier.natural]->setIndex(i);
    }
}

void GlobalMonitor::waypointCallBack(const mutac_msgs::Identifier &msg) {
    if (drones.find(msg.natural) != drones.end())
        drones[msg.natural]->advanceWP();
    else
        cout << "WARNING: Msg received for a drone that doesn't exists or is lost." << endl;
}

void GlobalMonitor::eventCallBack(const mutac_msgs::State &msg) {
    if (drones.find(msg.identifier.natural) != drones.end() || lost_drones.find(msg.identifier.natural) != lost_drones.end())
        if (msg.state == msg.FINISHED) {
            drones[msg.identifier.natural]->setState(State::MISSION_FINISHED);
            drones[msg.identifier.natural]->advanceWP();
        }
        else if (msg.state == msg.LANDED) {
            drones[msg.identifier.natural]->setState(State::LANDED);
        }
        else if (msg.state == msg.LOST) {
            drones[msg.identifier.natural]->saveState();
            drones[msg.identifier.natural]->setState(State::LOST);
            drones[msg.identifier.natural]->setPosInTrj(vector<double>{msg.position.x, msg.position.y, msg.position.z});
        } else {
            lost_drones[msg.identifier.natural]->reset();
            //pose_subs.push_back(nh.subscribe("drone" + to_string(msg.identifier.natural+1) + "/drone_pose", 1, &GlobalDrone::positionCallBack, lost_drones[msg.identifier.natural].get()));
            drones[msg.identifier.natural] = lost_drones[msg.identifier.natural];
            drones[msg.identifier.natural]->setIndex(drones.size()-1);
            lost_drones.erase(msg.identifier.natural);
            askReplan();
        }
    else
        cout << "WARNING: Event received for a drone that doesn't exists." << endl;
}
