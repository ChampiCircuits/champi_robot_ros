#!/bin/sh

# Say that this script must be run from the root of the workspace (ask and exit if not)
echo "This script must be run from the root of the workspace AND without sudo."
echo "Do you want to continue? (y/n)"
read answer
if [ "$answer" != "y" ]; then
  echo "Exiting..."
  exit 1
fi

# Ask if using bash or zsh and set a variable accordingly
echo "Are you using bash or zsh (type bash or zsh)?"
read shell
if [ "$shell" = "bash" ]; then
  shellrc=".bashrc"
elif [ "$shell" = "zsh" ]; then
  shellrc=".zshrc"
else
  echo "Exiting..."
  exit 1
fi

# Ask if ARM architecture is being used and set a variable accordingly
echo "Are you using an ARM architecture (e.g., Raspberry Pi)? (y/n)"
read arm
if [ "$arm" = "y" ]; then
  arm="arm"
else
  arm=""
fi

# Put the workspace in the environment variable CHAMPI_WS_DIR in .zshrc (check if it already exists first)
if grep -q "CHAMPI_WS_DIR" ~/$shellrc; then
  echo "The environment variable CHAMPI_WS_DIR already exists."
else
  echo "export CHAMPI_WS_DIR=$(pwd)" >> ~/$shellrc
fi

# Export for use in this script
export CHAMPI_WS_DIR=$(pwd)

# Create an alias to call champi_build.sh from any directory (check if it already exists first)
if grep -q "alias champi_build" ~/$shellrc; then
  echo "The alias champi_build already exists."
else
  echo "alias champi_build='$CHAMPI_WS_DIR/src/champi_robot_ros/scripts/champi_build.sh'" >> ~/$shellrc
fi

# Say that the command champi_build can be used from any directory
echo "The command champi_build can now be used from any directory."


# Install python dependencies
echo "Installing python dependencies..."
pip3 install -r $CHAMPI_WS_DIR/src/champi_robot_ros/requirements.txt
# Install robotpy (command differ for ARM architecture)
if [ "$arm" = "arm" ]; then
  python3 -m pip install --extra-index-url=https://wpilib.jfrog.io/artifactory/api/pypi/wpilib-python-release-2024/simple robotpy
else
  pip3 install robotpy
fi
echo "Python dependencies installed."

# Install rosdep dependencies
echo "Installing rosdep dependencies..."
sudo apt install -y python3-rosdep2
# TODO installer rosdep si ce n'est pas déjà fait
sudo rosdep init
rosdep update
rosdep install --from src --ignore-src -y

# Install apt dependencies
echo "Installing APT dependencies..."
sudo apt update
xargs -a $CHAMPI_WS_DIR/src/champi_robot_ros/apt-requirements.txt sudo apt install -y
echo "APT dependencies installed."

# Check if the repository ldlidar_stl_ros2 already exists
if [ -d "$CHAMPI_WS_DIR/src/ldlidar_stl_ros2" ]; then
  echo "The repository ldlidar_stl_ros2 already exists."
  echo "Do you want to delete it and clone it again? (y/n)"
  read answer
  if [ "$answer" != "y" ]; then
    echo "Setup Done!"
    exit 0
  fi
  rm -rf $CHAMPI_WS_DIR/src/ldlidar_stl_ros2
  git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git $CHAMPI_WS_DIR/src/ldlidar_stl_ros2
else
  echo "Cloning ldlidar_stl_ros2..."
  git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git $CHAMPI_WS_DIR/src/ldlidar_stl_ros2
fi




# Check if repositories rviz_birdeye_display spatz_interfaces already exist
if [ -d "$CHAMPI_WS_DIR/src/rviz_birdeye_display" ] || [ -d "$CHAMPI_WS_DIR/src/spatz_interfaces" ]; then
  echo "The repositories rviz_birdeye_display and/or spatz_interfaces already exist."
  echo "Do you want to delete them and clone them again? (y/n)"
  read answer
  if [ "$answer" != "y" ]; then
    echo "Setup Done!"
    exit 0
  fi
  rm -rf $CHAMPI_WS_DIR/src/rviz_birdeye_display
  rm -rf $CHAMPI_WS_DIR/src/spatz_interfaces
  git clone https://github.com/teamspatzenhirn/rviz_birdeye_display.git $CHAMPI_WS_DIR/src/rviz_birdeye_display
  git clone https://github.com/teamspatzenhirn/spatz_interfaces.git $CHAMPI_WS_DIR/src/spatz_interfaces
  exit 0
fi

echo "Setup Done!"
