#!/bin/bash
# exit when any command fails
set -e

# FIXME is not executable at this state, but just lists necesarry installation steps.

sudo apt install curl
# Install ROS 2 Eloquent and its dependencies
sudo apt update -y && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt update -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update -y
sudo apt install ros-eloquent-desktop -y
sudo apt install ros-eloquent-launch-xml
sudo apt install python3-colcon-common-extensions -y


# Install joy package: https://index.ros.org/p/joy/
sudo apt-get install ros-eloquent-joy

# force feedback using Logitech G29: Install https://github.com/kuriatsu/ros-g29-force-feedback\
mkdir -p ./ros2_ws/src
cd ./ros2_ws/src
git clone https://github.com/kuriatsu/ros-g29-force-feedback.git
cd ../
source /opt/ros/eloquent/setup.bash
colcon build --symlink-install