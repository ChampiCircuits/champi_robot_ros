#!/bin/sh

# Install python dependencies
echo "Installing python dependencies..."
pip3 install -r ~/champi_ws/src/champi_robot_ros/requirements.txt --break-system-packages

# Install rosdep dependencies
echo "Installing rosdep dependencies..."
sudo apt install -y python3-rosdep2
sudo rosdep init
rosdep update
rosdep install --from src --ignore-src -y

# Install apt dependencies
echo "Installing APT dependencies..."
sudo apt update
xargs -a ~/champi_ws/src/champi_robot_ros/apt-requirements.txt sudo apt install -y
echo "APT dependencies installed."

echo "Cloning other repositories...(throws error if already exists)"
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git ~/champi_ws/src/ldlidar_stl_ros2
git clone https://github.com/ChampiCircuits/rviz_static_image_display.git ~/champi_ws/src/rviz_static_image_display


echo "Setup Done!"
