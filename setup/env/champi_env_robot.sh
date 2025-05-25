#!/bin/sh

#source this file in .bashrc or .zshrc

source ~/champi_ws/src/champi_robot_ros/setup/env/env_common.sh

alias chapi='~/champi_ws/src/champi_robot_ros/setup/robot/tmux/champi_start.bash'
alias chapo='~/champi_ws/src/champi_robot_ros/setup/robot/tmux/champi_stop.bash'
alias attach='~/champi_ws/src/champi_robot_ros/setup/robot/tmux/champi_attach.bash'
alias launch='~/champi_ws/src/champi_robot_ros/setup/robot/tmux/champi_launch.bash'
alias champystem='sudo systemctl start champystem.service'
alias champystop='sudo systemctl stop champystem.service'
