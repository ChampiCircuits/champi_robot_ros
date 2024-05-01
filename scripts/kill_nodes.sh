#!/bin/bash

# List of nodes to kill
nodes=("base_controller_simu" "robot_state_publisher" "pub_goal_rviz" "twist_mux" "simu_lidar_node")

# Loop over the nodes and kill each one
for node in "${nodes[@]}"
do
    pkill -f $node
done