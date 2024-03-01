#!/bin/sh

# Bash script that call the command "colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1"
# from the directory specified by the env variable CHAMPI_WS_DIR. The users can pass other arguments to the
# colcon build command. Then, the script copies the compile_commands.json from the build directory to the path
# CHAMPI_WS_DIR/src/champi_robot_ros/compile_commands.json


# Check if the env variable CHAMPI_WS_DIR is set
if [ -z "$CHAMPI_WS_DIR" ]; then
    echo "The environment variable CHAMPI_WS_DIR is not set. Please set it to the path of the workspace directory."
    exit 1
fi

# Check if the directory CHAMPI_WS_DIR exists
if [ ! -d "$CHAMPI_WS_DIR" ]; then
    echo "The directory CHAMPI_WS_DIR does not exist."
    exit 1
fi

# Check if the directory CHAMPI_WS_DIR/src/champi_robot_ros exists
if [ ! -d "$CHAMPI_WS_DIR/src/champi_robot_ros" ]; then
    echo "The directory CHAMPI_WS_DIR/src/champi_robot_ros does not exist."
    exit 1
fi

# Call the colcon build command
cd $CHAMPI_WS_DIR
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 $@

# Check if the directory CHAMPI_WS_DIR/build exists
if [ ! -d "$CHAMPI_WS_DIR/build" ]; then
    echo "The directory CHAMPI_WS_DIR/build does not exist."
    exit 1
fi

# Copy the compile_commands.json from the build directory to the path CHAMPI_WS_DIR/src/champi_robot_ros/compile_commands.json
cp $CHAMPI_WS_DIR/build/compile_commands.json $CHAMPI_WS_DIR/src/champi_robot_ros/compile_commands.json

# Exit
exit 0
