cmake_minimum_required(VERSION 3.0.2)
project(multi_robot_simulator)

set(Flightmare_LIBS
    flightlib
    autopilot
    quadrotor_common
    nav_msgs
    eigen_catkin
    quadrotor_msgs
    polynomial_trajectories
    tf
    tf_conversions
    eigen_conversions
    image_transport
    cv_bridge
    mav_msgs
)

find_package(catkin REQUIRED mutac_msgs ${Flightmare_LIBS})
find_package(OpenCV REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

# Setup Default Build Type as Release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

# Setup Architcture-specific Flags
if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
    message(STATUS "Using ARMv7 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
elseif ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    message(STATUS "Using ARM aarch64 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
else ()
    set(CMAKE_CXX_ARCH_FLAGS -march=native)
endif ()

# Setup General C++ Flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=1048576")
# otherwise double free or corruption (out) error when running racing or motion_planning example
add_compile_options(-O3)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS 
)

add_library(unexpected_events src/unexpected_events/event_generator.cpp src/unexpected_events/event_controller.cpp 
src/unexpected_events/event.cpp src/unexpected_events/drone.cpp)
target_link_libraries(unexpected_events ${catkin_LIBRARIES})

add_executable(multi_robot_simulator src/simulator/flightmare_simulator.cpp)

# Setup Release and Debug flags
target_compile_options(multi_robot_simulator PRIVATE
$<$<CONFIG:Debug>:-Wall -g>
$<$<CONFIG:Release>:${CMAKE_CXX_ARCH_FLAGS} -Wall -DNDEBUG -fPIC>)

target_link_libraries(multi_robot_simulator
unexpected_events ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} stdc++fs zmq zmqpp)
