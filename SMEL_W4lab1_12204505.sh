#!/bin/bash

### SMEL Week 4 Lab file 12204505 Guliza Aitkulova
### Learning and following the tutorials on CLI tools from the ROS website

### PART 1 - Configuring the ROS environment

#command to set up the ROS source
source /opt/ros/foxy/setup.bash

#adding the command to the shell startup script
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

#to check the environment
printenv | grep -i ROS

#setting my own domain name
export ROS_DOMAIN_ID=18

#adding it to the shell startup script
echo "export ROS_DOMAIN_ID=18" >> ~/.bashrc

#to avoid intersections with other computers on the local network
export ROS_LOCALHOST_ONLY=1

#again addin it to the shell startup script
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

### PART 2 - Using turtlesim,ros2, and rqt

#installing the turtlesim node
sudo apt update
sudo apt install ros-foxy-turtlesim

#checking the installed package
ros2 pkg executables turtlesim

#running the turtlesim
ros2 run turtlesim turtlesim_node

#to control the turtle
ros2 run tutrlesim turtle_teleop_key

#installing the user-friendly GUI for the turtlesim
sudo apt update
sudo apt install ~nros-foxy-rqt*

#running the rqt
rqt
 
### PART 3 - Understanding nodes

#checking the running nodes after running the turtlesim
ros2 node list
#the output now is "/turtlesim"

#checking the running nodes after running the teleop
ros2 node list
#now the list is "/turtlesim  teleop_turtle"

#renaming the turtle
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
#if we check the node list now - the list also has "/my_turtle"

#to check the information of a specific node - for example here my_turtle
ros2 node info /my_turtle
#we can also request the other node's info to compare their subscribers, services, actions

### PART 4 - Understanding topics

#running the turtlesim with the teleop
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
#now our turtle can move

#opening the rqt graph for visualizing
rqt_graph
#the graph window is showing the nodes and their communication

#checking the list of topics
ros2 topic list

#displaying the topic list with topic type
ros2 topic list -t

#checking the data that is being published by the teleop for example
ros2 topic echo /turtle1/cmd_vel
#to see the output we need to go to the teleop window and use the arrow keys to move the turtle
#when I unchecked the debug parameter in the rqt graph I could see the /rqt_gui_py_node_26706 created by the echo command above

#another way of seeing the topic's information
ros2 topic info /turtle1/cmd_vel

#the cmd_vel has the type geometry_msgs/msg/Twist
#now can learn the details further

#then we can publish data onto a topic directly with the following command template
ros2 topic pub <topic_name> <msg_type> '<args>'
#here we are going to use this command like this
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

#when executing the following command the turtle draws a circle and goes in circles
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
#here we removes the --once which stops the turtle from moving firther

#rechecking the rqt graph
ros2 topic echo /turtle1/pose

#checking the rate at which data is being published
ros2 topic hz /turtle1/pose
 
### PART 5 - Understanding services

#opening a new terminal and running thr turtlesim and teleop nodes
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

#checking the currently active services
ros2 service list

#checking the service type
ros2 service type /clear

#seeing all types of all the services at the same time
ros2 service list -t

#finding all the services of a specific type
ros2 service find std_srvs/srv/Empty
ros2 interface show turtlesim/srv/Spawn

#calling an example service from the command line
ros2 interface show std_srvs/srv/Empty.srv

#calling a service command
ros2 service call <service_name> <service_type> <arguments>
ros2 service call /clear std_srvs/srv/Empty

#calling a spawning service
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"

### PART 6 - Understanding parameters

#again running the turltlesim and the teleop

#checking the parameter list
ros2 param list

#seeing the type and the current value of a parameter
ros2 param get <node_name> <parameter_name>
ros2 param get /turtlesim background_g
ros2 param get /turtlesim background_b
ros2 param get /turtlesim background_r

#changing a parameter's value at runtime
ros2 param set <node_name> <parameter_name> <value>

ros2 param set /turtlesim background_r 150
#this command above changed the background color to purple

#we can save a node's current parameter values
ros2 param dump <node_name>

ros2 param dump /turtlesim

#loading parameters from a file to a currently running node
ros2 param load <node_name> <parameter_file>
ros2 param load /turtlesim ./turtlesim.yaml

#starting the same node using my saved parameter values
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>

#stopping the running turtlesim node and reloading it with the saved parameters
ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml

### PART 7 - Understanding actions

#again running the turtlesim and the teleop
#to understand the actions - I played with the pressing different letters arounf the F key
#I also noticed the output changes in the turtlesim terminal window

#seeing the actions provided by a node
ros2 node info /turtlesim
ros2 node info /teleop_turtle

#checking the list of all actions
ros2 action list

#checking the action types
ros2 action list -t

#inverstigating a specific action further
ros2 action info /turtle1/rotate_absolute

#checking the structure of an action
ros2 interface show turtlesim/action/RotateAbsolute

#sending an action from the command line
ros2 action send_goal <action_name> <action_type> <values>
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"

#adding --feedback to see the feedback of the goal
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback

### PART 8 - Using rqt_console to view logs

#running the rqt_console
ros2 run rqt_console rqt_console

#running the turtlesim again
#then making the turtle hit the wall
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"
#we see warning messages in the rqt_console because the turtle is running into the wall continuously

#setting the default logger level
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN
#from this part of the tutorial I learned about the log messages in the rqt_console

### PART 9 - Launching nodes

#opened a new terminal window and launched 2 turtlesim windows
ros2 launch turtlesim multisim.launch.py

#opened two additional terminal windows and made the two turtles rotate in opposite directions
#the first turtle
ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
#the second turtle
ros2 topic pub  /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"

### PART 10 - Recording and playing back data

#installing the ros2 bag which is a command line tool for recording data published on topics in my system
sudo apt-get install ros-foxy-ros2bag \
                     ros-foxy-rosbag2-converter-default-plugins \
                     ros-foxy-rosbag2-storage-default-plugins

#making a new directory to save our recording
mkdir bag_files
cd bag_files

#checking the topics
ros2 topic list

#seeing the data of the /turtle1/cmd_vel topic
ros2 topic echo /turtle1/cmd_vel
#returning to the teleop window and moving the turtle to see output of the command above

#recording the data published to a topic
ros2 bag record <topic_name>
# I went to the bag_files directory
ros2 bag record /turtle1/cmd_vel
#the command above is starting to record the topic
#i opened the teleop window and moved the turtle around and then stopped the command above
#when I checked the directory with ls -l command, I founf a new file with the recorded data

#I can record multiple topics
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose
#the -o option helps me to give a name to the file with the recordings

#opening the recordings file
ros2 bag info <bag_file_name>
ros2 bag info subset

#I stopped the teleop window and entered the following command to play the recorded filed
ros2 bag play subset
#the turtle in the newly opened turtlesim moved in the same way that I did before because I recorded my topic

#seeing the rate at which data is being published
ros2 topic hz /turtle1/pose


