#!/bin/sh

# Bash script that call the command "colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1"
# from the directory specified by the env variable CHAMPI_WS_DIR. The users can pass other arguments to the
# colcon build command.
# Then, the script copies the compile_commands.json from the build directory to our the root
# of our champi_robot_ros. This is for clion!

# Call the colcon build command
cd ~/champi_ws
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 $@

# Copy the compile_commands.json from the build directory to the path champi_robot_ros/compile_commands.json
cp  ~/champi_ws/build/compile_commands.json  ~/champi_ws/src/champi_robot_ros/compile_commands.json
