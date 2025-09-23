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

echo "🍄 Launching script to compile and send STM firmware to the robot (stm32h7x) through the mini pc..."

# Determine which IP to use
if is_reachable $IP_ETH; then
    ROBOT_IP=$IP_ETH
elif is_reachable $IP_WIFI; then
    ROBOT_IP=$IP_WIFI
else
    echo "🛑 Robot not reachable at $IP_WIFI or $IP_ETH"
    exit 1
fi

# STM firmware compilation
echo "\n🔧 Building STM firmware"
mkdir -p "$STM_MAIN_DIR/build"
cd "$STM_MAIN_DIR/build" || exit 1

# These options were taken from STM32CubeIDE project settings
cmake .. \
  -DCMAKE_C_FLAGS="-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb" \
  -DCMAKE_CXX_FLAGS="-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb" \
  -DCMAKE_ASM_FLAGS="-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb"

make -j"$(nproc)"
cd -

# Check if the local ELF differs from the one on the robot
echo "\n🔍 Checking if the firmware ELF changed..."
rsync -azvn "$STM_ELF_PATH" "$USER@$ROBOT_IP:$REMOTE_ELF_PATH" | grep -q "$STM_ELF_NAME" # dry run

if [ $? -eq 0 ]; then
    echo "\n🚚 Firmware changed, syncing ELF to the robot..."
    rsync -az "$STM_ELF_PATH" "$USER@$ROBOT_IP:$REMOTE_ELF_PATH"

    echo "\n🚀 Remotely flashing STM32 firmware on the robot..."
    ssh "$USER@$ROBOT_IP" << EOF
      set -e
      cd "$DEST_PATH/stm_main_board/build"
      openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program $STM_ELF_NAME verify reset exit"
EOF

    if [ $? -eq 0 ]; then
        echo "\n🎉 Flash completed successfully on the robot!"
    else
        echo "\n🛑 Flash failed on the robot."
    fi
else
    echo "\n🛑 Firmware is already up to date, no flashing needed."
fi
