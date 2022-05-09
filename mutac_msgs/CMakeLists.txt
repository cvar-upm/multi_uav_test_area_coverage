cmake_minimum_required(VERSION 3.0.2)
project(mutac_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

add_message_files(
  FILES
  Alarm.msg
  Battery.msg
  Generation.msg
  Identifier.msg
  Label.msg
  LabeledPath.msg
  LabeledPoint.msg
  Metrics.msg
  Plan.msg
  Sweep.msg
)

add_service_files(
  FILES
  GeneratePlan.srv
  UpdatePlan.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})
