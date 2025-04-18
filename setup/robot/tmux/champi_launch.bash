#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/champi/champi_ws/install/setup.bash

ros2 launch champi_bringup bringup.launch.py sim:=false teleop:=true