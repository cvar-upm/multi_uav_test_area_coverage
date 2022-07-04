# multi_uav_test_area_coverage

A multi UAV test platform for area coverage path planning algorithms.

## Installation

### Install Flightmare

The `multi_robot_simulator` uses [Flightmare](https://github.com/uzh-rpg/flightmare) to simulate the mission. To use it, you need to install [Flightmare with ROS](https://github.com/uzh-rpg/flightmare/wiki/Install-with-ROS).

After installing it, if you want to use the solar farm scene used in the examples, you need to install the modified Flightmare binary from here (link), extract it into the */path/to/flightmare/flightrender*, and modify the following files:

- *flightmare/flightlib/include/flightlib/bridges/unity_message_types.hpp*

```cpp
// In UnityScene Enum add the ID for the solar farm scene and increase the number of scenes

enum UnityScene {
  INDUSTRIAL = 0,
  WAREHOUSE = 1,
  GARAGE = 2,
  TUNELS = 4,
  NATUREFOREST = 3,
  SOLARFARM = 5,
  // total number of environment
  SceneNum = 6
};
```

- *rpg_quadrotor_common/quadrotor_common/include/quadrotor_common/trajectory_point.h*

```cpp
// Add the definition of the copy constructor
TrajectoryPoint(const TrajectoryPoint& other);
```

- *rpg_quadrotor_common/quadrotor_common/src/trajectory_point.cpp*

```cpp
// Implement the copy constructor

TrajectoryPoint::TrajectoryPoint(const TrajectoryPoint& other)
: time_from_start(other.time_from_start)
, position(other.position)
, orientation(other.orientation)
, velocity(other.velocity)
, acceleration(other.acceleration)
, jerk(other.jerk)
, snap(other.snap)
, bodyrates(other.bodyrates)
, angular_acceleration(other.angular_acceleration)
, angular_jerk(other.angular_jerk)
, angular_snap(other.angular_snap)
, heading(other.heading)
, heading_rate(other.heading_rate)
, heading_acceleration(other.heading_acceleration)
{}
```

- *rpg_quadrotor_control/trajectory_planning/polynomial_trajectories/include/polynomial_trajectories/polynomial_trajectory.h*

```cpp
// Add the definition of the copy constructor
PolynomialTrajectory(const PolynomialTrajectory& other);
```

- *rpg_quadrotor_control/trajectory_planning/polynomial_trajectories/src/polynomial_trajectory.cpp*

```cpp
// Implement the copy constructor

PolynomialTrajectory::PolynomialTrajectory(const PolynomialTrajectory& other)
: trajectory_type(other.trajectory_type)
, coeff(other.coeff)
, T(other.T)
, start_state(other.start_state)
, end_state(other.end_state)
, number_of_segments(other.number_of_segments)
, segment_times(other.segment_times)
, optimization_cost(other.optimization_cost)
{}
```

### Install PySide6

Install the PySide6 library with the following command if you want to use the `performance_viewer`:

```
pip install pyside6
```

### MUTAC workspace

Create a catkin workspace with:

```
cd
mkdir -p catkin_ws/src
```

Now, inside the workspace you can clone the MUTAC repository.

```
cd ~/catkin_ws/src
git clone https://github.com/cvar-upm/multi_uav_test_area_coverage
```

Add the workspace setup to the *.bashrc* file and create te **MUTAC_WS** environment variable for the workspace.

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export MUTAC_WS=~/catkin_ws/src" >> ~/.bashrc
source ~/.bashrc
```

Now, build the workspace:

```
catkin build
```

After this installation you can now execute every example in the `test` package.
