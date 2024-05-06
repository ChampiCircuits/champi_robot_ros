#!/bin/bash

# List of nodes to kill
nodes=("base_controller_simu" "robot_state_publisher" "pub_goal_rviz" "twist_mux" "simu_lidar_node" "robot_stopped_detector" "ukf" "static_transform_publisher_map_odom")
# nodes=("base_controller_simu" "robot_state_publisher" "pub_goal_rviz" "twist_mux" "simu_lidar_node" "robot_stopped_detector" "ukf" "static_transform_publisher_map_odom")

# Loop over the nodes and kill each one
for node in "${nodes[@]}"
do
    pkill -f $node
done