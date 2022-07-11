# Multi UAV test area coverage

A multi UAV test platform for area coverage path planning algorithms.

## Introduction

The multi UAV test area coverage repository is a set of tools that is part of an experimental platform aimed at evaluating different replanning algorithms for multi-agent flight systems.

It consists of the following components:

- *planner*: plans a mission given a region to inspect and the number of drones to use.

- *execution_monitor*: drone monitors that detect important events such as the drone getting lost or finishing the inspection.

- *performance_evaluator*: generates useful metrics such as the total time of the mission or the total distance covered.

- *performance_viewer*: graphic interface that shows the current state of the mission and its metrics.

- *multi_robot_simulator*: simulates the mission in a photorealistic environment using [Flightmare](https://github.com/uzh-rpg/flightmare). It also simulates any unexpected events that can occur to a drone. The events currently supported are: BLIND_CAMERA, GO_HOMEBASE, BATTERY_DISCHARGE, FALL_DOWN, STOPPED, RECTILINEAR_MOTION, SLOW_MOTION.

## Installation

### Install Flightmare

The `multi_robot_simulator` uses [Flightmare](https://github.com/uzh-rpg/flightmare) to simulate the mission. To use it, you need to install [Flightmare with ROS](https://github.com/uzh-rpg/flightmare/wiki/Install-with-ROS).

After installing it, if you want to use the solar farm scene used in the examples, you need to install the modified Flightmare binary from here (link), extract it into the */path/to/flightmare/flightrender* and modify the following file:

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

To use multiple quadrotors in Flightmare you also need to change the following files:

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

### Install dependencies

Install the PySide6 library with the following command if you want to use the `performance_viewer`, and `tmux` to execute the examples:

```
pip install pyside6
sudo apt install tmux
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
git clone https://github.com/cvar-upm/multi_uav_test_area_coverage .
```

Build the workspace:

```
catkin build
```

To finish, add the workspace setup to the *.bashrc* file and create the **MUTAC_WS** environment variable for the workspace.

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export MUTAC_WS=~/catkin_ws/src" >> ~/.bashrc
source ~/.bashrc
```

After this installation you can now execute every example in the `test` package.

## Usage

### Running Tests

The framework exposes several mission cases for testing multiarea coverage planners. These cases are located in the `tests/n_drones/casei` directory, `n` the size of the tested swarm and `i` the test case.

Each test is composed of a `config.yaml` and `examples_events.csv` files defining the mission and a `mission_launch.sh` launch script. Executing the `mission_launch.sh` script initializes the test. For instance, the test case 1 for 3 agents invocation would look like:

```
$ ./tests/3_drones/case1/mission_launch.sh
```

On mission initialization, a `tmux` session is created with different tabs for the `planner`, `multi_robot_simulator`, `performance_evaluation`, `performance_viewer`, `global_monitor` and `execution_monitor`s.

The `performance_viewer` instance shows a graphic interface with mission data and metrics while the `multi_robot_simulator` renders the simulation scenario and the camera view of each agent.

Mission will not start until the `planner`'s `generate_plan` service is invoked with the input region. This starts the system with the `config.yaml` parameters.

Once the mission is finished, the `tmux` session can be killed with `Ctrl` + `B` and typing `:kill-sesssion` in the terminal.

### Modifying Tests

New tests can be created by modifying existing ones. The `config.yaml` file inside a test case contains the starting swarm, the location of the events file and launch parameters of the planner. An events file holds all the unexpected events that will happen during the mission. By default it is the `example_events.csv` next to the `config.yaml` file in each test case.

The availiable entries in the `config.yaml` are:

| Entry                           | Type    | Description                 | Example                   |
|---------------------------------|---------|-----------------------------|---------------------------|
| `n_drones`                      | Integer | Size of the initial swarm   | `3`                       |
| `homebase/dronei`               | List    | Home of agent `i`           | `[590, 90, 325]`          |
| `mission/height`                | Float   | Inspection Height           | `370`                     |
| `mission/distance`              | Float   | Path Separation             | `10`                      |
| `common_space/height`           | Float   | Positioning Base Height     | `375`                     |
| `common_space/height_increment` | Float   | Positioning Safety Distance | `5`                       |
| `update_plan/threshold`         | Float   | Replanning Threshold        | `0.8`                     |
| `events/file_path`              | String  | Events File                 | `/tmp/example_events.csv` |

The events file contains a sheet with all the events that may happen during execution. Each event declaration specifies the target agent, the firing instant and the event itself with its corresponding parameters. A discharge of a 5% of the agent with identifier 1 after 15 seconds of execution would be represented as:

```
1,15,BATTERY_DISCHARGE,5
```

The availiable events are:

| Event                | Parameter  | Description                                        |
|----------------------|------------|----------------------------------------------------|
| `BATTERY_DISCHARGE`  | Percentage | Reduces the batery of the agent by the percentage. |
| `FALL_DOWN`          | None       | Forces the agent to fall down.                     |
| `RECTILINEAR_MOTION` | None       | Forces the agent to move in a line.                |
| `STOPPED`            | Time       | Forces the agent to stop for the specified time.   |
| `GO_HOMEBASE`        | None       | Forces the agent to return home.                   |
| `BLIND_CAMERA`       | None       | Deactivates the agent camera.                      |
| `SLOW_MOTION`        | None       | Reduces the agent movement speed.                  |
