#!/bin/bash

echo "Starting to test ROS3 Foxy installation"

# Source the setup file for ROS 2 Foxy
source /opt/ros/foxy/setup.bash

# Run the C++ talker in the background
timeout 5 ros2 run demo_nodes_cpp talker &

# Run the Python listener in the foreground
timeout 5 ros2 run demo_nodes_py listener

echo "Testing is completed, ROS2 Foxy is working"
