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

    trj_sub = nh.subscribe("planned_paths", 100, &GlobalMonitor::trajectoryCallBack, this);
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
                
                pose_subs[droneID].shutdown();
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

        ros::spinOnce();
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

        point.label.natural = (path.points[0].label != drone->getLastLabel()) ? mutac_msgs::Label::POSITIONING_LABEL : path.points[0].label.natural;
        path.points.insert(path.points.begin(), point);

        paths[drone->getIndex()] = path;

        it++;
    }

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
    if (drones.find(msg.identifier.natural) != drones.end())
        if (msg.state == msg.FINISHED) {
            drones[msg.identifier.natural]->setState(State::MISSION_FINISHED);
            drones[msg.identifier.natural]->advanceWP();
        }
        else if (msg.state == msg.LANDED) {
            drones[msg.identifier.natural]->setState(State::LANDED);
        }
        else {
            drones[msg.identifier.natural]->saveState();
            drones[msg.identifier.natural]->setState(State::LOST);
            drones[msg.identifier.natural]->setPosInTrj(vector<double>{msg.position.x, msg.position.y, msg.position.z});
        }
    else
        cout << "WARNING: Event received for a drone that doesn't exists or is lost." << endl;
}
