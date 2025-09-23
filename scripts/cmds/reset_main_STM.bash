#!/bin/bash

# Configuration
USER="champi"
DEST_PATH="/home/champi/champi_ws/src/champi_robot_ros"
LOCAL_PATH="$HOME/champi_ws/src/champi_robot_ros"
IP_ETH="10.0.0.1"
IP_WIFI="172.0.0.1"
STM_MAIN_DIR="$LOCAL_PATH/stm_main_board"
STM_ELF_NAME="stm_main_board.elf"
STM_ELF_PATH="$STM_MAIN_DIR/build/$STM_ELF_NAME"
REMOTE_ELF_PATH="$DEST_PATH/stm_main_board/build/$STM_ELF_NAME"

# Function to check if a host is reachable
is_reachable() {
    ping -c 1 -W 1 $1 > /dev/null 2>&1
    return $?
}


# Determine which IP to use
if is_reachable $IP_ETH; then
    ROBOT_IP=$IP_ETH
elif is_reachable $IP_WIFI; then
    ROBOT_IP=$IP_WIFI
else
    echo "🛑 Robot not reachable at $IP_WIFI or $IP_ETH"
    exit 1
fi

echo "🍄 Remotely resetting the stm on the robot"

ssh "$USER@$ROBOT_IP" << EOF
  set -e
  cd "$DEST_PATH/stm_main_board/build"
  openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "init; reset halt; reset run; exit"
EOF

if [ $? -eq 0 ]; then
    echo "\n🎉 Flash completed successfully on the robot!"
else
    echo "\n🛑 Flash failed on the robot."
fi