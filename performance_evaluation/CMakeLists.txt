cmake_minimum_required(VERSION 3.0.2)
project(performance_evaluation)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  mutac_msgs
)

catkin_python_setup()

catkin_package(
  LIBRARIES performance_evaluator
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs mutac_msgs
)

include_directories(performance_evaluator/include ${catkin_INCLUDE_DIRS})

add_library(performance_evaluator performance_evaluator/src/drone.cpp)
target_link_libraries(performance_evaluator ${catkin_LIBRARIES})

add_executable(metricsGenerator performance_evaluator/src/metricsGenerator.cpp)
target_link_libraries(metricsGenerator ${catkin_LIBRARIES} performance_evaluator)

catkin_install_python(PROGRAMS
  performance_viewer/scripts/display.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)