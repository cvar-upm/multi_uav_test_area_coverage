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
  Context.msg
  Event.msg
  EventIdentified.msg
  Identifier.msg
  PathIdentified.msg
  Region.msg
  RegionIdentified.msg
  Metrics.msg
)

add_service_files(
  FILES
  Split.srv
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
