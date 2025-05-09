#!/bin/sh

alias build='~/champi_ws/src/champi_robot_ros/scripts/cmds/champi_build.sh'
alias clean='rm -R ~/champi_ws/log ~/champi_ws/build ~/champi_ws/install'
alias kill_nodes='~/champi_ws/src/champi_robot_ros/scripts/cmds/kill_nodes.bash'

export RCUTILS_COLORIZED_OUTPUT=1

# argcomplete for ros2 & colcon
eval "$(register-python-argcomplete ros2)"
eval "$(register-python-argcomplete colcon)"