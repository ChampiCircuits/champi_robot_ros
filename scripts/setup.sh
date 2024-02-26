#!/bin/sh


# Install dependencies
sudo apt update
sudo apt install -y libprotobuf-dev

cd src
git clone https://github.com/teamspatzenhirn/rviz_birdeye_display.git
git clone https://github.com/teamspatzenhirn/spatz_interfaces.git
cd ..