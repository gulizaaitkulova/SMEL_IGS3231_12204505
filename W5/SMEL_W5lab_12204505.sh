### SMEL W5 Lab - 12204505
### ROS Tutorials - Intermediate

# PART 1 - Managing Dependencies with rosdep

# initializing rosdep and updating the locally cached rosdistro index
sudo rosdep init
rosdep update

# installing dependencies
rosdep install --from-paths src -y --ignore-src

# Creating an action

mkdir -p ros2_ws/src #you can reuse existing workspace with this naming convention
cd ros2_ws/src
ros2 pkg create action_tutorials_interfaces

