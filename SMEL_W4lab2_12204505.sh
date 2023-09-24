#!/bin/bash

### SMEL Week 4 Lab session part 2 - Guliza Aitkulova 12204505
### Client libraries

### PART 1 - Using colcon to build packages

#installing colcon
sudo apt install python3-colcon-common-extensions

#creating a directory to contain my workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

#cloning the examples repository into the src directory of the workspace
git clone https://github.com/ros2/examples src/examples -b foxy

#installing symlink
colcon build --symlink-install

#testing the packages
colcon test

#sourcing
source install/setup.bash

#trying out demos with the subscriber and a publisher
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
ros2 run examples_rclcpp_minimal_publisher publisher_member_function

#adding the changing directory comman to the startup shell file
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/foxy/" >> ~/.bashrc

echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

### PART 2 - Creating a workspace
#sourcing the ros
source /opt/ros/foxy/setup.bash 

#creating a new directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

#cloning the repository
git clone https://github.com/ros/ros_tutorials.git -b foxy-devel

### here at this point I had a problem with checking the dependies installed
#every time I would get an error about the packages of the ROS distro and rosdep
#but after surfing the internet looking for solutions I could find the folowing command to solve my problem - I was stuck here about 10-15 minutes trying the command again and again reentering the commands
rosdep update --include-eol-distros

rosdep install -i --from-path src --rosdistro foxy -y

#so now I had the following message finally
#All required rosdeps installed successfully

#building the packages
colcon build

#here I encountered problems but i could do it all over again starting from the colcon installation and then I could move on

#next I opened a new terminal and started buildin an overlay
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws

#sourcing the overlay
source install/local_setup.bash

### PART 3 - Creating a package

#going to the directory
cd ~/ros2_ws/src

#creating a new package
ros2 pkg create --build-type ament_cmake <package_name>

ros2 pkg create --build-type ament_cmake --node-name my_node my_package

#returned to the root workspace and built the packages
colcon build

#building only my_package
colcon build --packages-select my_package

#sourcing
source install/local_setup.bash

#running the executable which returns hello world
ros2 run my_package my_node

#then I opened the package.xml file and edited the maintainer, description, and license lines and saved them

