#!/bin/bash

### 12204505 Guliza Aitkulova
### Smart Mobility Engineering Lab - Week 6 lab - ROS tutorials
### Advanced - Part 2 - Using Fast DDS Discovery Server as discovery protocol [community-contributed]


# seeting up a discovery server
fastdds discovery --server-id 0

# launching listener node 
# setting the environment variable
export ROS_DISCOVERY_SERVER=127.0.0.1:11811

ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server

# running the talker node
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server

# demonstrating discovery server execution
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=simple_listener

ros2 run demo_nodes_cpp talker --ros-args --remap __node:=simple_talker

# establishing a communication with rebundant servers
fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811
fastdds discovery --server-id 1 --ip-address 127.0.0.1 --port 11888


# running talker-listener again
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker

export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

# backup server
fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811 --backup

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

# running the first server listening on localhost with default port of 11811
fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811

fastdds discovery --server-id 1 --ip-address 127.0.0.1 --port 11888

export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_1

export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_1

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_2

export ROS_DISCOVERY_SERVER=";127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_2

