cmake_minimum_required(VERSION 3.15.0)

project(planner VERSION 1.0.0)



find_package(catkin
    REQUIRED
    COMPONENTS
    mutac_msgs
    roscpp
)



catkin_package(
    CATKIN_DEPENDS
    mutac_msgs
    roscpp
)



add_subdirectory(src/multi_uav_planning_binpat)



add_executable(planner src/main.cpp)

add_dependencies(planner ${catkin_EXPORTED_TARGETS})

target_include_directories(planner PRIVATE src ${catkin_INCLUDE_DIRS})

target_link_libraries(planner planning_binpat ${catkin_LIBRARIES})
