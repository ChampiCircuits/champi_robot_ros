#!/bin/bash

/usr/bin/tmux new-session -d -s champi_session /home/champi/champi_ws/src/champi_robot_ros/setup/robot/tmux/champi_launch.bash
/usr/bin/tmux new-session -d -s web_session /home/champi/champi_ws/src/champi_robot_ros/setup/robot/tmux/champi_web.bash
/usr/bin/tmux new-session -d -s foxglove_session "/bin/bash -c 'source /opt/ros/jazzy/setup.bash; ros2 run foxglove_bridge foxglove_bridge'"