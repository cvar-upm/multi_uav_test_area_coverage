# multi_uav_test_area_coverage
A multi UAV test platform for area coverage path planning algorithms.

## Messages

| Topic name | ROS message | Description |
|----------|-------------|------|
| planned_path | [mutac_msgs/PathIdentified](https://github.com/cvar-upm/multi_uav_test_area_coverage/blob/main/mutac_msgs/msg/PathIdentified.msg) | Path to follow by a drone |
| drone_pose | [geometry_msgs/Pose](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html) | Current drone pose |
| drone_twist | [geometry_msgs/Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) | Current drone twist |
| cover_region | [mutac_msgs/RegionIdentified](https://github.com/cvar-upm/multi_uav_test_area_coverage/blob/main/mutac_msgs/msg/RegionIdentified.msg) | Region to be covered by a certain drone |
| performance_metrics | [mutac_msgs/Metrics](https://github.com/cvar-upm/multi_uav_test_area_coverage/blob/main/mutac_msgs/msg/Metrics.msg) | Performance metrics used to monitor the mission  | 
| event | [mutac_msgs/EventIdentified](https://github.com/cvar-upm/multi_uav_test_area_coverage/blob/main/mutac_msgs/msg/EventIdentified.msg) | Event raised by a drone during mission execution |
| simulated_event | [mutac_msgs/EventIdentified](https://github.com/cvar-upm/multi_uav_test_area_coverage/blob/main/mutac_msgs/msg/EventIdentified.msg) | Event raised by external means during mission execution | 


## Services
| Service name | ROS service | Description |
|----------|-------------|------|
| split | [mutac_msgs/Split](https://github.com/cvar-upm/multi_uav_test_area_coverage/blob/main/mutac_msgs/srv/Split.srv) | Divides a region into subregions taking into account a specific context |
