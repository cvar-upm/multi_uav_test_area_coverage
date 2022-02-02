# multi_uav_test_area_coverage
A multi UAV test platform for area coverage path planning algorithms.

## Messages

| Topic name | ROS message | Description |
|----------|-------------|------|
| planned_path | mutac_msgs/PathIdentified | Path to follow by a drone |
| drone_pose | geometry_msgs/Pose | Current drone pose |
| drone_twist | geometry_msgs/Twist | Current drone twist |
| cover_region | mutac_msgs/Region | Region to be covered by a certain drone |
| event | mutac_msgs/EventIdentified | Event raised by a drone during mission execution |
| simulated_event | mutac_msgs/EventIdentified | Event raised by external means during mission execution | 


## Services
| Service name | ROS service | Description |
|----------|-------------|------|
| split | mutac_msgs/Split | Divides a region into subregions taking into account a specific context |