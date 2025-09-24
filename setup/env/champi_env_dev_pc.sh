#!/bin/sh

#source this file in .bashrc or .zshrc

source ~/champi_ws/src/champi_robot_ros/setup/env/env_common.sh

alias share_internet='~/champi_ws/src/champi_robot_ros/scripts/cmds/share_internet.sh'
alias rsync_update='~/champi_ws/src/champi_robot_ros/scripts/cmds/rsync_ros.bash'
alias serial_monitor='pio device monitor -p /dev/ttyACM0 -b 115200 -f direct' # TODO changer la regle udev pour qu'il s'appelle toujours le même nom ?
alias champiros='$(export ROS_DOMAIN_ID=0)'
