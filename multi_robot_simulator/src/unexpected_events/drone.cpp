#include "unexpected_events/drone.h"

Drone::Drone(int id, std::vector<double> init_pos) {
    this->id = id;
    battery = 100;
    position = init_pos;

    stopped_time = 0;
    event_stop_time = 0;
    start_stop = ros::Time::now();

    state = std::pair<MotionState, CameraState> {MotionState::NOT_STARTED, CameraState::NO_CAMERA};
    prev_state = state;
    
    trj_changed = false;
    trj_points = std::vector<std::vector<double>>();

    timer = ros::Time::now();;

    acceleration << 0.0, 0.0, 0.0;
    velocity << 0.0, 0.0, 0.0;
}

Drone::Drone(const Drone &other) :
    id(other.id), battery(other.battery), position(other.position), stopped_time(other.stopped_time), 
    event_stop_time(other.event_stop_time), start_stop(other.start_stop), state(other.state), 
    prev_state(other.prev_state), trj_changed(other.trj_changed), trj_points(other.trj_points), 
    trajectory(other.trajectory), event_trajectory(other.event_trajectory), trj_velocity(other.trj_velocity), 
    trj_thrust(other.trj_thrust), acceleration(other.acceleration), velocity(other.velocity), timer(other.timer) {}

/* Returns the point of the trajectory for the specified time.
Returns an empty vector if the drone has not started yet, it has already landed or it as stopped. */
std::vector<double> Drone::getNextPoint(ros::Time time) {
    std::vector<double> point;
    ros::Duration duration = ros::Duration(time - timer) - ros::Duration(stopped_time);
    
    std::shared_ptr<PolynomialTrajectory> trj = state.first == MotionState::FOLLOWING_PLAN || state.first == MotionState::SLOW_MOTION ? 
    trajectory : event_trajectory;
    
    if (state.first == MotionState::NOT_STARTED || state.first == MotionState::STOPPED || state.first == MotionState::FINISHED) {
        return point;
    }

    if (duration > trj->T) {
        state.first = MotionState::FINISHED;
        velocity << 0.0, 0.0, 0.0;
        return point;
    }
    
    if (changeTrajectory()) {
        trj_points.erase(trj_points.begin());

        std::vector<Eigen::Vector3d> points;
        points.push_back(Eigen::Vector3d {position[0], position[1], position[2]});
        points.push_back(Eigen::Vector3d {trj_points[0][0], trj_points[0][1], trj_points[0][2]});
        
        generateTrj(points, false);
        trj_changed = true;
        stopped_time = 0;
    }
    
    quadrotor_common::TrajectoryPoint desired_pose = getPointFromTrajectory(*trj, duration);
    
    velocity = desired_pose.velocity;
    acceleration = desired_pose.acceleration;

    point = std::vector<double> {desired_pose.position.x(), desired_pose.position.y(), 
                                desired_pose.position.z()};
    
    return point;
}

/* Generates the trajectory between two points. */
void Drone::generateTrj(std::vector<Eigen::Vector3d> points, bool event) {
    Eigen::VectorXd segment_times(1);
    Eigen::VectorXd minimization_weights(1);
    minimization_weights << 1.0;
    segment_times << calculateSegmentTime(points[0], points[1]);;

    quadrotor_common::TrajectoryPoint start_state;
    quadrotor_common::TrajectoryPoint end_state;
    end_state.position = points[1];
    start_state.position = points[0];

    PolynomialTrajectorySettings trajectory_settings =
    PolynomialTrajectorySettings(decltype(points)(), minimization_weights, 8, 3);
    
    PolynomialTrajectory trj = 
    minimum_snap_trajectories::generateMinimumSnapTrajectory(segment_times, start_state, end_state, trajectory_settings, trj_velocity, trj_thrust, 6.0);

    if(event)
        event_trajectory = std::make_shared<PolynomialTrajectory>(trj);
    else
        trajectory = std::make_shared<PolynomialTrajectory>(trj);
}

// Detects if the trajectory has to change to the next one.
bool Drone::changeTrajectory() {
    std::vector<double> point {trj_points[0][0], trj_points[0][1], trj_points[0][2]};

    double distance = std::sqrt(std::pow((position[0] - point[0]), 2) + std::pow((position[1] - point[1]), 2) + std::pow((position[2] - point[2]), 2)); 
    return distance <= 1 && trj_points.size() > 1;
}

double Drone::calculateSegmentTime(Eigen::Vector3d p1, Eigen::Vector3d p2) {
    double dist = std::sqrt(std::pow((p1[0] - p2[0]), 2) + std::pow((p1[1] - p2[1]), 2) + std::pow((p1[2] - p2[2]), 2));
    return dist / trj_velocity;
}
