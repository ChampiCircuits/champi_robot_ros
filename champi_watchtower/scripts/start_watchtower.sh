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
colcon build --packages-select champi_vision --symlink-install

if [ $? -ne 0 ]; then
    echo "❌ Build failed!"
    exit 1
fi

echo "✓ Build successful"
echo ""

# Source the workspace
source ~/champi_ws/install/setup.bash

echo "Available launch options:"
echo ""
echo "1. Launch with defaults (uses config files from package):"
echo "   ros2 launch champi_vision watchtower.launch.py"
echo ""
echo "2. Launch with custom parameters:"
echo "   ros2 launch champi_vision watchtower.launch.py \\"
echo "     camera_info_file:=/path/to/camera.yaml \\"
echo "     table_width:=3.0 \\"
echo "     marker_length:=0.07"
echo ""
echo "3. Launch in Webots simulation mode:"
echo "   ros2 launch champi_vision watchtower.launch.py \\"
echo "     is_simu_with_webots:=true"
echo ""

# Ask user if they want to launch now
read -p "Launch watchtower node now with defaults? (y/N): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "🚀 Launching watchtower node..."
    echo ""
    echo "Instructions:"
    echo "1. A GUI window will appear"
    echo "2. Click 'Start Calibration' when ready"
    echo "3. The node will calibrate and start localizing robots"
    echo ""
    echo "Press Ctrl+C to stop"
    echo ""
    ros2 launch champi_vision watchtower.launch.py
else
    echo ""
    echo "Skipped launch. Run manually when ready."
fi

echo ""
echo "For more information, see:"
echo "  ~/champi_ws/src/champi_robot_ros/champi_vision/champi_vision/watchtower/README_WATCHTOWER_NODE.md"
