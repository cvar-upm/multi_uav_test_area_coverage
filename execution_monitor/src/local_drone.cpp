#include "execution_monitor/monitor_info/local_drone.h"

LocalDrone::LocalDrone(int id) : MonitorData(id) {
    battery = 100;
    camera = true;
    deviated = false;

    waypoints = vector<vector<double>>();
    inspection_wps = vector<vector<double>>();

    prev_distance = DBL_MAX;
    lastWP = position;
}

/* Checks the state of the drone.
The drone is identified as lost if it deviates from its trajectory, it falls down, the camera failed 
or the battery is below 5%.
Return values:
    -1: the drone is okay
    0: the drone has finished
    1: the drone got lost
    2: the drone landed
    3: the drone passed through a waypoint
*/
int LocalDrone::checkDrone(double distTrj, double distWP) {
    // If drone has landed, is lost or hasn't started yet, dont check anything
    if(state == State::NOT_STARTED) return -1;

    // Camera or Battery failure
    if (!camera || battery <= 5) {
        state = State::LOST;
        return 1;
    }

    vector<vector<double>> wps = state == State::ON_MISSION ? inspection_wps : waypoints;

    // Calculate distance to waypoint
    double waypoint_dist = distance(position, wps[0]);

    // Check if the drone is finishing the inspection or landing
    if (wps.size() == 1 && waypoint_dist <= distWP) {
        advanceWP();
        
        // If the drone was going back, it landed
        if (state == State::GOING_HOME) {
            state = State::LANDED;
            return 2;
        }
        // Else, the drone finished the inspection
        else {
            state = State::GOING_HOME;
            return 0;
        }
    }

    // Check if the drone is passing through the next waypoint
    if (waypoint_dist < distWP) {
        advanceWP();
        return 3;
    }
    
    // Check if drone is following the trajectory correctly
    if(distanceToTrj(position, lastWP, wps[0]) > distTrj) {
        pos_in_trj = calculateProjection(position, lastWP, wps[0]);
        deviated = true;
        state = State::LOST;
        return 1;
    }
    
    // Check if drone is moving forward
    if (waypoint_dist > prev_distance && fabs(waypoint_dist - prev_distance) > distTrj) {
        deviated = true;
        state = State::LOST;
        return 1;
    }

    if (waypoint_dist < prev_distance) {
        prev_distance = waypoint_dist;
        pos_in_trj = position;
    }

    return -1;
}

// Advances the waypoints lists in one
void LocalDrone::advanceWP() {
    prev_distance = DBL_MAX;
    lastWP = waypoints[0];
    waypoints.erase(waypoints.begin());
    if(inspection_wps.size() > 0) inspection_wps.erase(inspection_wps.begin());
}

/* -------------
ROS Callbacks
------------- */ 

void LocalDrone::batteryCallBack(const sensor_msgs::BatteryState &msg) {
    battery = msg.percentage;
}

/* ----------------
Waypoints setters
---------------- */ 

void LocalDrone::setWaypoints(vector<vector<double>> waypoints) {
    waypoints.erase(waypoints.begin());
    this->waypoints = waypoints; 
    
    lastWP = position;
    prev_distance = DBL_MAX;    
}

void LocalDrone::setInspectionWPS(vector<vector<double>> inspection_wps) {
    inspection_wps.erase(inspection_wps.begin());
    this->inspection_wps = inspection_wps;
}

/* ---------------------------
Auxiliar geometry functions
--------------------------- */ 

double LocalDrone::vectorNorm(double x, double y, double z) {
    return sqrt(pow((x), 2) + pow((y), 2) + pow((z), 2));
}

// Distance between two points
double LocalDrone::distance(vector<double> p1, vector<double> p2) {
    return vectorNorm(p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]);
}

// Distance from a point to the trajectory
double LocalDrone::distanceToTrj(vector<double> pos, vector<double> p1, vector<double> p2) {
    vector<double> AB {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
    vector<double> AP {pos[0] - p1[0], pos[1] - p1[1], pos[2] - p1[2]};

    double i = AB[1] * AP[2] - AP[1] * AB[2];
    double j = AP[0] * AB[2] - AB[0] * AP[2];
    double k = AB[0] * AP[1] - AB[1] * AP[0];
    
    return vectorNorm(i, j, k) / vectorNorm(AB[0], AB[1], AB[2]);
}

// Position projection on the trajectory
vector<double> LocalDrone::calculateProjection(vector<double> pos, vector<double> wp1, vector<double> wp2) {
    // OA + ((AM . u) / |u|^2) x u
    vector<double> AM = vector<double> {pos[0] - wp1[0], pos[1] - wp1[1], pos[2] - wp1[2]};
    vector<double> u = vector<double> {wp2[0] - wp1[0], wp2[1] - wp1[1], wp2[2] - wp1[2]};

    double scalar = (AM[0] * u [0] + AM[1] * u [1] + AM[2] * u [2]) / pow(vectorNorm(u[0], u[1], u[2]), 2);
    
    return vector<double> {wp1[0] + scalar * u[0], wp1[1] + scalar * u[1], wp1[2] + scalar * u[2]};
}
