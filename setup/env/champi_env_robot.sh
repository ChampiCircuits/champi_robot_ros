#!/bin/sh

#source this file in .bashrc or .zshrc

source ~/champi_ws/src/champi_robot_ros/setup/env/env_common.sh

alias bringup_restart='sudo systemctl restart champi.service'
alias bringup_status='systemctl status champi.service'
alias bringup_stop='sudo systemctl stop champi.service'
alias bringup_attach='journalctl -u champi.service --since "$(systemctl show -p ActiveEnterTimestamp --value champi.service)" -r'
alias bringup_direct_start='sudo systemctl stop champi.service &&  ~/champi_ws/src/champi_robot_ros/setup/robot/tmux/champi_launch.bash'
alias champystem='sudo systemctl start champystem.service'
alias champystop='sudo systemctl stop champystem.service'
alias serial_monitor0='pio device monitor -p /dev/ttyACM0 -b 115200 -f direct'
alias serial_monitor1='pio device monitor -p /dev/ttyACM1 -b 115200 -f direct'
