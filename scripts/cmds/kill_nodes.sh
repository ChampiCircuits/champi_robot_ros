#!/bin/sh

# TODO Check if still works + get nodes automatically

# List of nodes to kill
nodes=("base_controller_simu" "robot_state_publisher" "pub_goal_rviz" "twist_mux" "simu_lidar_node" "robot_stopped_detector" "ukf" "static_transform_publisher_map_odom" "base_controller" "imu_controller" "v4l2_camera_node" "sm_ros_itf")
# Loop over the nodes and kill each one
for node in "${nodes[@]}"
do
    pkill -f $node
done