cmake_minimum_required(VERSION 3.0.2)
project(execution_monitor)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp  
  geometry_msgs
  nav_msgs
  sensor_msgs
  mutac_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES execution_monitor
  CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs mutac_msgs roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(execution_monitor_lib src/global_drone.cpp src/local_drone.cpp src/monitor_data.cpp)
target_link_libraries(execution_monitor_lib ${catkin_LIBRARIES})

add_executable(execution_monitor src/execution_monitor.cpp)
target_link_libraries(execution_monitor ${catkin_LIBRARIES} execution_monitor_lib)

add_executable(global_execution_monitor src/global_monitor.cpp)
target_link_libraries(global_execution_monitor ${catkin_LIBRARIES} execution_monitor_lib)