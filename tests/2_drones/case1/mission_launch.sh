#!/bin/bash

SESSION=$USER
CONFIG_FILE="$MUTAC_WS/tests/2_drones/case1/config.yaml"
RVIZ_FILE="$MUTAC_WS/tests/2_drones/camera2.rviz"

rosparam load $CONFIG_FILE mutac

tmux -2 new-session -d -s $SESSION

tmux new-window -t $SESSION:1 -n 'Planning service'
tmux send-keys "roslaunch planner planner.launch --wait" C-m

tmux new-window -t $SESSION:2 -n 'Flightmare Simulator'
tmux send-keys "roslaunch multi_robot_simulator multi_robot_simulator.launch rviz_file:=$RVIZ_FILE --wait" C-m

tmux new-window -t $SESSION:3 -n 'Performance Viewer'
tmux send-keys "roslaunch performance_evaluation performance_viewer.launch --wait" C-m

tmux new-window -t $SESSION:4 -n 'Performance Evaluator'
tmux send-keys "roslaunch performance_evaluation performance_evaluator.launch --wait" C-m

tmux new-window -t $SESSION:5 -n 'Global Monitor'
tmux send-keys "roslaunch execution_monitor global_monitor.launch --wait" C-m

tmux new-window -t $SESSION:6 -n 'Execution Monitor Drone 1'
tmux send-keys "roslaunch execution_monitor execution_monitor.launch drone_id:=1 --wait" C-m

tmux new-window -t $SESSION:7 -n 'Execution Monitor Drone 2'
tmux send-keys "roslaunch execution_monitor execution_monitor.launch drone_id:=2 --wait" C-m

tmux attach-session -t $SESSION:1