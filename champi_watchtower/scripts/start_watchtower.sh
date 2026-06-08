#!/bin/bash
# Quick start script for watchtower node
# This script helps you get started quickly with the watchtower system

set -e

echo "=========================================="
echo "Watchtower Node Quick Start"
echo "=========================================="
echo ""

# Check if workspace is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not sourced! Please run:"
    echo "   source /opt/ros/<distro>/setup.bash"
    echo "   source ~/champi_ws/install/setup.bash"
    exit 1
fi

echo "✓ ROS2 environment detected: $ROS_DISTRO"
echo ""

# Build the package
echo "Building champi_vision package..."
cd ~/champi_ws
colcon build --packages-select champi_vision champi_watchtower --symlink-install

if [ $? -ne 0 ]; then
    echo "❌ Build failed!"
    exit 1
fi

echo "✓ Build successful"
echo ""

# Source the workspace
source ~/champi_ws/install/setup.sh

echo ""
echo "🚀 Launching watchtower node..."
echo ""
ros2 launch champi_watchtower watchtower.launch.py is_simu_with_webots:=True
# TODO by default launch not in simu !