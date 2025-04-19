#!/bin/sh

alias build='~/champi_ws/src/champi_robot_ros/scripts/cmds/champi_build.sh'
alias kill_nodes='~/champi_ws/src/champi_robot_ros/scripts/cmds/kill_nodes.bash'

export RCUTILS_COLORIZED_OUTPUT=1

# argcomplete for ros2 & colcon
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"