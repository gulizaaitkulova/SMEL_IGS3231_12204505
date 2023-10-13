#!/bin/bash

### 12204505 Guliza Aitkulova
### Smart Mobility Engineering Lab - Week 6 lab - ROS tutorials
### Advanced - Part 1 - Enabling topic statistics (C++)

#navigating to the necessary folder
cd ros2_ws/src/cpp_pubsub/src

#downloading the example talker code but I will comment the line because the link doesn't work
# wget -O member_function_with_topic_statistics.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_subscriber/member_function_with_topic_statistics.cpp

#instead we will need to download a cpp file manually - I will incldue the file in this folder
# you will need to manually paste W6_1.2/member_function_with_topic_statistics.cpp file into ros2_ws/src/cpp_pubsub/src

# adding the listener_with_topic_statistics executable
cd ~/ros_2/src/cpp_pubsub
echo "add_executable(listener_with_topic_statistics src/member_function_with_topic_statistics.cpp)
ament_target_dependencies(listener_with_topic_statistics rclcpp std_msgs)

install(TARGETS
  talker
  listener
  listener_with_topic_statistics
  DESTINATION lib/${PROJECT_NAME})" > CMakeLists.txt

# now we will need to run the nodes and see the statistics