#!/bin/sh

# Put the workspace in the environment variable CHAMPI_WS_DIR in .zshrc
echo "export CHAMPI_WS_DIR=$(pwd)" >> ~/.zshrc

# Export for use in this script
export CHAMPI_WS_DIR=$(pwd)

# Create an alias to call champi_build.sh from any directory
echo "alias champi_build='$CHAMPI_WS_DIR/src/champi_robot_ros/scripts/champi_build.sh'" >> ~/.zshrc

# Say that the command champi_build can be used from any directory
echo "The command champi_build can now be used from any directory."

# Install rosdep dependencies
echo "Installing rosdep dependencies..."
rosdep install --from src --ignore-src -y

# Install dependencies
echo "Installing dependencies..."
sudo apt update
sudo apt install -y libprotobuf-dev

# Clone repositories needed to display images in rviz
echo "Cloning other ros packages..."
git clone https://github.com/teamspatzenhirn/rviz_birdeye_display.git $CHAMPI_WS_DIR/src/rviz_birdeye_display
git clone https://github.com/teamspatzenhirn/spatz_interfaces.git $CHAMPI_WS_DIR/src/spatz_interfaces
