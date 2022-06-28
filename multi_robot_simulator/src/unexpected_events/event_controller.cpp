#include "unexpected_events/event_controller.h"

EventController::EventController(std::shared_ptr<Drone> drone, std::vector<double> homebase, double floorZ) :
    drone(drone), homebase(homebase), floorZ(floorZ) {
        nh = ros::NodeHandle("");
        alarm_pub = nh.advertise<mutac_msgs::Alarm>("drone_alarm", 100);
        battery_pub = nh.advertise<sensor_msgs::BatteryState>("drone" + to_string(drone->getId()+1) + "/battery_state", 100);
}

EventController::EventController(const EventController &other) :
    drone(other.drone), homebase(other.homebase), floorZ(other.floorZ), timer(other.timer), nh(other.nh), battery_pub(other.battery_pub),
    alarm_pub(other.alarm_pub) {}

void EventController::startEvent(Event event) {
    switch (event.getType()) {
    case EventType::BLIND_CAMERA:
        std::cout << "Drone: " << drone->getId() << " Event: BLIND_CAMERA" << std::endl;
        blindCamera();
        break;
    case EventType::GO_HOMEBASE:
        std::cout << "Drone: " << drone->getId() << " Event: GO_HOMEBASE" << std::endl;
        goHomeBase();
        break;
    case EventType::BATTERY_DISCHARGE:
        std::cout << "Drone: " << drone->getId() << " Event: BATTERY_DISCHARGE" << std::endl;
        batteryDischarge(event.getParam());
        break;
    case EventType::FALL_DOWN:
        std::cout << "Drone: " << drone->getId() << " Event: FALL_DOWN" << std::endl;
        fallDown();
        break;
    case EventType::STOPPED:
        std::cout << "Drone: " << drone->getId() << " Event: STOPPED" << std::endl;
        stop(event.getParam());
        break;
    case EventType::RECTILINEAR_MOTION:
        std::cout << "Drone: " << drone->getId() << " Event: RECTILINEAR_MOTION" << std::endl;
        rectilinearMotion();
        break;
    case EventType::SLOW_MOTION:
        std::cout << "Drone: " << drone->getId() << " Event: SLOW_MOTION" << std::endl;
        slowMotion();
        break;
    case EventType::WRONG_EVENT:
        std::cout << "Drone: " << drone->getId() << " Event: WRONG_EVENT" << std::endl;
        slowMotion();
        break;
    }
}

void EventController::timerCallback(const ros::TimerEvent&) {
    drone->setState(drone->getPrevState());
}

/* ------
Events
------ */

void EventController::blindCamera() {
    drone->setCameraState(CameraState::CAMERA_FAILURE);
    mutac_msgs::Alarm msg = mutac_msgs::Alarm();
    msg.identifier.natural = drone->getId();
    msg.camera = false;
    alarm_pub.publish(msg);
}

void EventController::goHomeBase() {
    drone->setMotionState(MotionState::GOING_HOMEBASE);
    
    Eigen::Vector3d start {drone->getPosition()[0], drone->getPosition()[1], drone->getPosition()[2]};
    Eigen::Vector3d end {homebase[0], homebase[1], homebase[2]};
    std::vector<Eigen::Vector3d> points {start, end};

    drone->generateTrj(points, true);

    drone->setTrjChanged(true);
}

void EventController::batteryDischarge(int percentage) {    
    sensor_msgs::BatteryState msg = sensor_msgs::BatteryState();
    msg.percentage = drone->getBattery() - percentage;
    
    if (msg.percentage <= 0) {
        msg.percentage = 0;
        fallDown();
    }

    drone->setBattery(msg.percentage);
    battery_pub.publish(msg);
}

void EventController::fallDown() {
    drone->setMotionState(MotionState::FALLING_DOWN_FAILURE);
    
    Eigen::Vector3d start(drone->getPosition()[0], drone->getPosition()[1], drone->getPosition()[2]);
    Eigen::Vector3d end(drone->getPosition()[0], drone->getPosition()[1], floorZ);
    std::vector<Eigen::Vector3d> points {start, end};

    drone->generateTrj(points, true);
    drone->setTrjChanged(true);
}

void EventController::stop(int secs) {
    drone->setPrevState(drone->getState());
    drone->setMotionState(MotionState::STOPPED);
    
    drone->setStoppedTime(drone->getStoppedTime() + secs);
    drone->setEventStopTime(secs);
    drone->setStartStop(ros::Time::now());

    drone->setVelocity(Eigen::Vector3d(0, 0, 0));

    timer = nh.createTimer(ros::Duration(secs), &EventController::timerCallback, this, true);
}

void EventController::rectilinearMotion() {
    drone->setMotionState(MotionState::RECTILINEAR_MOTION);
    
    Eigen::Vector3d start(drone->getPosition()[0], drone->getPosition()[1], drone->getPosition()[2]);
    Eigen::Vector3d end(drone->getPosition()[0] + 700, drone->getPosition()[1], drone->getPosition()[2]);
    std::vector<Eigen::Vector3d> points {start, end};

    drone->generateTrj(points, true);
    drone->setTrjChanged(true);
}

void EventController::slowMotion() {
    drone->setMotionState(MotionState::SLOW_MOTION);
    Eigen::Vector3d start(drone->getPosition()[0], drone->getPosition()[1], drone->getPosition()[2]);

    Eigen::Vector3d end(drone->getTrjPoints()[0][0], drone->getTrjPoints()[0][1], drone->getTrjPoints()[0][2]);
    std::vector<Eigen::Vector3d> points {start, end};

    drone->setTrjVelocity(5);
    drone->setTrjThrust(10);
    
    drone->generateTrj(points, false);
    drone->setTrjChanged(true);
}
