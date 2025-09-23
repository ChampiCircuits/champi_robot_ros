#!/bin/bash

USER="champi"
DEST_PATH="/home/champi/champi_ws/src/champi_robot_ros"
LOCAL_PATH="$HOME/champi_ws/src/champi_robot_ros"
STM_PAMI_DIR="$LOCAL_PATH/stm_pami_board"
STM_ELF_NAME="stm_pami_board.elf"

set -e  # Exit immediately if a command exits with a non-zero status

echo "🍄 Launching script to compile and send STM firmware to a PAMI (stm32g4x)"

# STM firmware compilation
echo "\n🔧 Building STM firmware"
mkdir -p "$STM_PAMI_DIR/build"
cd "$STM_PAMI_DIR/build" || exit 1

cmake ..

make -j"$(nproc)"
cd -


echo "\n🚀 Flashing STM32 firmware on the pami..."
set -e
cd "$DEST_PATH/stm_pami_board/build"
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program $STM_ELF_NAME verify reset exit"


if [ $? -eq 0 ]; then
    echo "\n🎉 Flash completed successfully on the robot!"
else
    echo "\n🛑 Flash failed on the robot."
fi