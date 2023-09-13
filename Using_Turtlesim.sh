#!/bin/bash

echo "Starting SMEL Week3 Lab Simple Project with TurtleSim (Extra Activity)"

# Source the setup file for ROS 2 Foxy
source /opt/ros/foxy/setup.bash

# Update 
sudo apt update

# Install TurtleSim
sudo apt install ros-foxy-turtlesim

# Check if TurtleSim is installed
ros2 pkg executables turtlesim

# Run turtlesim_node in the background
ros2 run turtlesim turtlesim_node &

# Run turtle_teleop_key
ros2 run turtlesim turtle_teleop_key 

echo "Finishing SMEL Week3 Lab Simple Project with TurtleSim (Extra Activity)"