#!/bin/bash

### 12204505 Guliza Aitkulova
### Smart Mobility Engineering Lab - Week 6 lab - ROS tutorials
### Writing a simple publisher and subscriber (C++) - Part 1.1

cd /ros2_ws/src/cpp_pubsub

echo "cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)


install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()" > CMakeLists.txt

# going to the root of the workspace
cd ~/ros2_ws

rosdep install -i --from-path src --rosdistro foxy -y

colcon build --packages-select cpp_pubsub
