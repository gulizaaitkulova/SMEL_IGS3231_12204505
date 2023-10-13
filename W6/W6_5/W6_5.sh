#!/bin/bash

### 12204505 Guliza Aitkulova
### Smart Mobility Engineering Lab - Week 6 lab - ROS tutorials
### Advanced - Part 5 - Webots_ros2 installation

sudo apt-get install ros-foxy-webots-ros2

source /opt/ros/foxy/setup.bash

# setting a variable
export WEBOTS_HOME=/usr/local/webots

# sourcing
cd ~/ros2_ws
source install/local_setup.bash

# starting demo packages
ros2 launch webots_ros2_universal_robot multirobot_launch.py

