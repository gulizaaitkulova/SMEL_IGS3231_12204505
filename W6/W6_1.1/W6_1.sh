#!/bin/bash

### 12204505 Guliza Aitkulova
### Smart Mobility Engineering Lab - Week 6 lab - ROS tutorials
### Writing a simple publisher and subscriber (C++) - Part 1.1

# creating a workspace
mkdir ros2_ws
cd ros2_ws

mkdir src
cd src

# creating a package
ros2 pkg create --build-type ament_cmake cpp_pubsub

# downloading the example talker code
cd cpp_pubsub/src

wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_publisher/member_function.cpp

# downloading the example listener code
wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_subscriber/member_function.cpp

# the next step is to modify the package.xml file - but it should be done manually - it is explained in the README file. Please, follow the guidelines from the README file.