#include "execution_monitor/monitor_info/monitor_data.h"

MonitorData::MonitorData (int id) {
    this->id = id;
    state = State::NOT_STARTED;

    position = vector<double>(3, 0);
    pos_in_trj = vector<double>(3, 0);
}

void MonitorData::positionCallBack(const geometry_msgs::Pose &msg) {
    position[0] = msg.position.x;
    position[1] = msg.position.y;
    position[2] = msg.position.z;
}
