#include "simulator/flightmare_simulator.h"

using namespace flightlib;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "multi_robot_simulator");
    Simulator sim = Simulator();
    sim.start();

    return 0;
}

Simulator::Simulator() {
    nh = ros::NodeHandle("");
    pnh = ros::NodeHandle("~");

    int id;
    pnh.getParam("all_cameras", all_cameras);
    pnh.getParam("floor", floor);
    pnh.getParam("scene_id", id);
    SceneID scene_id = id;

    n_drones = 0;
    if (!pnh.getParam("/mutac/n_drones", n_drones)) {
        cerr << "\033[31;1mERROR: Missing parameter 'n_drones'.\033[0m" << endl;
        exit(1);
    }

    for (int i = 0; i < n_drones; i++) {
        std::string nspace = "/mutac/homebase/drone" + std::to_string(i+1);
        std::vector<double> pos;
        if (!nh.param<std::vector<double>>(nspace, pos, std::vector<double>{0, 0, 0})) {
            cerr << "\033[31;1mERROR: Missing parameter 'homebase/drone" << std::to_string(i+1) 
            << "'.\033[0m" << endl;
            exit(1);
        }
        homebases.push_back(pos);
    }

    // ROS communication
    connectROS();

    // Drones and event controllers
    configDrones();

    // Unity config
    configUnity(scene_id);

    // Event generator
    string path;
    if (!nh.getParam("/mutac/events/file_path", path))
        cerr << "\033[33;1mWARNING: Missing parameter 'events/file_path'.\033[0m" << endl;
    event_gen = EventGenerator(path);    
}

// Simulates the drones 
void Simulator::start() {
    FrameID frame_id = 1;
    ros::Rate rate(10);
    
    while (ros::ok() && unity_ready) {
        // Check to see if there are events to simulate
        std::vector<Event> events = event_gen.genEvents();
        for (size_t i = 0; i < events.size(); i++) {
            event_controllers[events[i].getDroneID() - 1].startEvent(events[i]);
        }
        
        restartTimer();
        ros::Time time = ros::Time::now();

        for (int i = 0; i < n_drones; i++) {
            // Trajectories
            QuadState quad_state = quad_states[i];
            std::vector<double> point = drones[i]->getNextPoint(time);
            
            if (!point.empty()) {
                std::shared_ptr<Quadrotor> quad_ptr = quad_ptrs[i];  
                quad_state.x[QS::POSX] = point[0];
                quad_state.x[QS::POSY] = point[1];
                quad_state.x[QS::POSZ] = point[2];

                quad_ptr->setState(quad_state);
                drones[i]->setPosition(point);

                if (event_gen.getStart() < 0) 
                    event_gen.setStart(ros::Time::now().toSec());
            }

            geometry_msgs::Pose msgPose = geometry_msgs::Pose();
            msgPose.position.x = drones[i]->getPosition()[0];
            msgPose.position.y = drones[i]->getPosition()[1];
            msgPose.position.z = drones[i]->getPosition()[2];
            pose_pubs[i].publish(msgPose);

            geometry_msgs::Twist msgTwist = geometry_msgs::Twist();
            msgTwist.linear.x = drones[i]->getVelocity()[0];
            msgTwist.linear.y = drones[i]->getVelocity()[1];
            msgTwist.linear.z = drones[i]->getVelocity()[2];
            speed_pubs[i].publish(msgTwist);
            
            // Cameras
            CameraState camera_state = drones[i]->getState().second;

            if (camera_state != CameraState::NO_CAMERA) {
                cv::Mat img;            

                if (camera_state == CameraState::CAMERA_OK)
                    rgb_cameras[i]->getRGBImage(img);
                else
                    img = cv::Mat(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));    

                sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
                rgb_msg->header.stamp = time;
                rgb_pubs[i].publish(rgb_msg);
            }
        }
        
        unity_bridge_ptr->getRender(frame_id);
        unity_bridge_ptr->handleOutput();
        
        frame_id += 1;
        ros::spinOnce();
        rate.sleep();
    }
}

// Restarts trajectory timers
void Simulator::restartTimer() {
    for (size_t i = 0; i < drones.size(); i++) {
        if (drones[i]->getTrjChanged()) {
            drones[i]->setTimer(ros::Time::now());
            drones[i]->setTrjChanged(false);
        }
    }
}

/* ----------------
ROS Communication
---------------- */

void Simulator::connectROS() {
    // Subscribers
    path_sub = nh.subscribe("planned_paths", 100, &Simulator::pathCallback, this);

    // Publishers
    for (int i = 0; i < n_drones; i++) {
        pose_pubs.push_back(nh.advertise<geometry_msgs::Pose>("drone" + std::to_string(i+1) + "/drone_pose", 100));
        speed_pubs.push_back(nh.advertise<geometry_msgs::Twist>("drone" + std::to_string(i+1) + "/drone_twist", 100));
    }
}

void Simulator::pathCallback(const mutac_msgs::Plan &msg) {
    for (size_t i = 0; i < msg.paths.size(); i++) {
        int droneID = msg.paths[i].identifier.natural;

        if ((size_t)droneID > drones.size() - 1 || droneID < 0) {
            std::cout << "WARNING: Trajectory received for a drone that doesn't exists." << std::endl;
            return;
        }
        
        std::vector<mutac_msgs::LabeledPoint> path = msg.paths[i].points;
        
        if (path.size() < 2) {
            std::cout << "WARNING: Trajectoy needs to have at leat 2 waypoints." << std::endl;
            return;
        }
        
        std::pair<MotionState, CameraState> state =  drones[droneID]->getState();

        std::vector<Eigen::Vector3d> points;
        std::vector<double> point;

        // Reset current plan
        drones[droneID]->clearTrjPoints();

        // Add the points of the new plan
        for (size_t i = 1; i < path.size(); i++) {
            drones[droneID]->addPoint(std::vector<double>{path[i].point.x, path[i].point.y, path[i].point.z});
        }
        
        // Generate the first trajectory
        // If the plan was received while on mission, 
        // the first point of the trajectory should be the current position
        if (state.first == MotionState::NOT_STARTED)
            point = std::vector<double> {path[0].point.x, path[0].point.y, path[0].point.z};
        else
            point = drones[droneID]->getPosition();
        
        points.push_back(Eigen::Vector3d {point[0], point[1], point[2]});     
        points.push_back(Eigen::Vector3d {path[1].point.x, path[1].point.y, path[1].point.z});

        drones[droneID]->generateTrj(points, false);

        // Start the drone and restart timer
        if(state.first == MotionState::NOT_STARTED) {
            drones[droneID]->setMotionState(MotionState::FOLLOWING_PLAN);
            drones[droneID]->setTrjChanged(true);
        }
        
        // Restart the timer if new trajectory
        else if (state.first == MotionState::FOLLOWING_PLAN || state.first == MotionState::SLOW_MOTION) {
            drones[droneID]->setTrjChanged(true);
            drones[droneID]->setStoppedTime(0);
        }

        // Restart stop time if new trajectory received while stopped
        else if (state.first == MotionState::STOPPED) {
            drones[droneID]->setTrjChanged(true);
            int stop = drones[droneID]->getEventStopTime() - (ros::Time::now() - drones[droneID]->getStartStop()).toSec();
            drones[droneID]->setStoppedTime(stop);
        }
    }   
}

/* ------
Config
------ */

void Simulator::configUnity(SceneID scene_id) {
    // Unity
    unity_bridge_ptr = UnityBridge::getInstance();
    unity_ready = false;

    Vector<3> quad_size(0.5, 0.5, 0.5);
    Vector<3> B_r_BC(0.0, 0.0, 0.0);
    Matrix<3, 3> R_BC = Quaternion(1.0, -0.75, 0.0, 0.0).toRotationMatrix();

    for (int i = 0; i < n_drones; i++) {
        // Quadrotors
        std::shared_ptr<Quadrotor> quad_ptr = std::make_shared<Quadrotor>(); 
        quad_ptr->setSize(quad_size);

        QuadState quad_state;
        quad_state.setZero();
        quad_state.x[QS::POSX] = homebases[i][0];
        quad_state.x[QS::POSY] = homebases[i][1];
        quad_state.x[QS::POSZ] = homebases[i][2];
        quad_ptr->reset(quad_state);
        
        quad_ptrs.push_back(quad_ptr);
        quad_states.push_back(quad_state);

        // Cameras
        if(all_cameras || drones[i]->getState().second == CameraState::CAMERA_OK) {
            image_transport::ImageTransport it(pnh);
            std::string pubName = "/rgb" + std::to_string(i+1);
            rgb_pubs[i] = it.advertise(pubName, 1);

            std::shared_ptr<RGBCamera> rgb_camera = std::make_shared<RGBCamera>();
            rgb_camera->setFOV(90);
            rgb_camera->setWidth(640);
            rgb_camera->setHeight(360);
            rgb_camera->setRelPose(B_r_BC, R_BC);
            quad_ptr->addRGBCamera(rgb_camera);
            rgb_cameras[i] = rgb_camera;
        }

        unity_bridge_ptr->addQuadrotor(quad_ptr);
    }

    unity_ready = unity_bridge_ptr->connectUnity(scene_id);
}

void Simulator::configDrones() {
    for (int i = 0; i < n_drones; i++) {
        drones.push_back(std::make_shared<Drone>(i, homebases[i]));
        EventController ev = EventController(drones[i], homebases[i], floor);
        event_controllers.push_back(ev);
        if(all_cameras)
            drones[i]->setCameraState(CameraState::CAMERA_OK);
    }

    if (!all_cameras) {
        std::vector<int> camera_ids;
        nh.getParam("/mutac/cameras", camera_ids);
        
        for (size_t i = 0; i < camera_ids.size(); i++) {
            drones[camera_ids[i]-1]->setCameraState(CameraState::CAMERA_OK);
        }
    }    
}
