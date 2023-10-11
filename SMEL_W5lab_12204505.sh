### SMEL W5 Lab - 12204505
### ROS Tutorials - Intermediate

# PART 1 - Managing Dependencies with rosdep

# initializing rosdep and updating the locally cached rosdistro index
sudo rosdep init
rosdep update

# installing dependencies
rosdep install --from-paths src -y --ignore-src

# PART 2 - Creating an action

# setting up a workspace and creating a package
mkdir -p ros2_ws/src #we can reuse existing workspace with this naming convention
cd ros2_ws/src
ros2 pkg create action_tutorials_interfaces

# creating action directory in our ROS 2 package
cd action_tutorials_interfaces
mkdir action

# creating a file called Fibonacci.action with the following contents:
int32 order
---
int32[] sequence
---
int32[] partial_sequence

# adding a piece of code which passes the definition to the rosidl code generation pipeline in the CMakeLists.txt
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)

# also adding the required dependencies to the package.xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>

# changing to the root of the workspace and building the package containing the Fibonacci action definition
# Change to the root of the workspace
cd ~/ros2_ws
# Build
colcon build

# checking that our action built successfully with the command line tool:
# Source our workspace
# On Windows: call install/setup.bat
. install/setup.bash
# Check that our action definition exists
ros2 interface show action_tutorials_interfaces/action/Fibonacci


# PART 3 - Writing an action server and client (Python)

# # created a python file called fibonacci_action_server.py with the following content
# import rclpy
# from rclpy.action import ActionServer
# from rclpy.node import Node

# from action_tutorials_interfaces.action import Fibonacci


# class FibonacciActionServer(Node):

#     def __init__(self):
#         super().__init__('fibonacci_action_server')
#         self._action_server = ActionServer(
#             self,
#             Fibonacci,
#             'fibonacci',
#             self.execute_callback)

#     def execute_callback(self, goal_handle):
#         self.get_logger().info('Executing goal...')
#         result = Fibonacci.Result()
#         return result


# def main(args=None):
#     rclpy.init(args=args)

#     fibonacci_action_server = FibonacciActionServer()

#     rclpy.spin(fibonacci_action_server)


# if __name__ == '__main__':
#     main()

# running the python file
python3 fibonacci_action_server.py

# in another terminal we are sending a goal
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"


# modifying the fibonaci_action_server.py file like so:
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        goal_handle.succeed()

        result = Fibonacci.Result()
        return result

# restarting the server file and sending a new goal
### result output image - SUCCEEDED

# modifying the server code so that it actually computes and returns the requested Fibonacci sequence:

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')


        sequence = [0, 1]



        for i in range(1, goal_handle.request.order):

            sequence.append(sequence[i] + sequence[i-1])


        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = sequence

        return result

# again restarting and sending another goal
### result output image

# again modifying the server file to replace the sequence variable, and use a feedback message to store the sequence instead


# import time


# import rclpy
# from rclpy.action import ActionServer
# from rclpy.node import Node

# from action_tutorials_interfaces.action import Fibonacci


# class FibonacciActionServer(Node):

#     def __init__(self):
#         super().__init__('fibonacci_action_server')
#         self._action_server = ActionServer(
#             self,
#             Fibonacci,
#             'fibonacci',
#             self.execute_callback)

#     def execute_callback(self, goal_handle):
#         self.get_logger().info('Executing goal...')


#         feedback_msg = Fibonacci.Feedback()

#         feedback_msg.partial_sequence = [0, 1]


#         for i in range(1, goal_handle.request.order):

# #             feedback_msg.partial_sequence.append(

#                 feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

#             self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))

#             goal_handle.publish_feedback(feedback_msg)

#             time.sleep(1)


#         goal_handle.succeed()

#         result = Fibonacci.Result()

#         result.sequence = feedback_msg.partial_sequence

#         return result


# def main(args=None):
#     rclpy.init(args=args)

#     fibonacci_action_server = FibonacciActionServer()

#     rclpy.spin(fibonacci_action_server)


# if __name__ == '__main__':
#     main()


# after restarting the server file we confirm that feedback is now published by using the command line tool with the --feedback option:
ros2 action send_goal --feedback fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

###result output image

# creating a new python file called fibonacci_action_client.py with the following content:
# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node

# from action_tutorials_interfaces.action import Fibonacci


# class FibonacciActionClient(Node):

#     def __init__(self):
#         super().__init__('fibonacci_action_client')
#         self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

#     def send_goal(self, order):
#         goal_msg = Fibonacci.Goal()
#         goal_msg.order = order

#         self._action_client.wait_for_server()

#         return self._action_client.send_goal_async(goal_msg)


# def main(args=None):
#     rclpy.init(args=args)

#     action_client = FibonacciActionClient()

#     future = action_client.send_goal(10)

#     rclpy.spin_until_future_complete(action_client, future)


# if __name__ == '__main__':
#     main()

# running both server and the client files in different terminal windows
python3 fibonacci_action_server.py
python3 fibonacci_action_client.py

### result output image

# modifying the client file content so that we know when the goal is completed
# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node

# from action_tutorials_interfaces.action import Fibonacci


# class FibonacciActionClient(Node):

#     def __init__(self):
#         super().__init__('fibonacci_action_client')
#         self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

#     def send_goal(self, order):
#         goal_msg = Fibonacci.Goal()
#         goal_msg.order = order

#         self._action_client.wait_for_server()

#         self._send_goal_future = self._action_client.send_goal_async(goal_msg)

#         self._send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Goal rejected :(')
#             return

#         self.get_logger().info('Goal accepted :)')

#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         result = future.result().result
#         self.get_logger().info('Result: {0}'.format(result.sequence))
#         rclpy.shutdown()


# def main(args=None):
#     rclpy.init(args=args)

#     action_client = FibonacciActionClient()

#     action_client.send_goal(10)

#     rclpy.spin(action_client)


# if __name__ == '__main__':
#     main()

# again running the server and client files
### result output image

# modifying the client file content so that we can get some feedback about the goals we send from the action server
# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node

# from action_tutorials_interfaces.action import Fibonacci


# class FibonacciActionClient(Node):

#     def __init__(self):
#         super().__init__('fibonacci_action_client')
#         self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

#     def send_goal(self, order):
#         goal_msg = Fibonacci.Goal()
#         goal_msg.order = order

#         self._action_client.wait_for_server()

#         self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

#         self._send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Goal rejected :(')
#             return

#         self.get_logger().info('Goal accepted :)')

#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         result = future.result().result
#         self.get_logger().info('Result: {0}'.format(result.sequence))
#         rclpy.shutdown()

#     def feedback_callback(self, feedback_msg):
#         feedback = feedback_msg.feedback
#         self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


# def main(args=None):
#     rclpy.init(args=args)

#     action_client = FibonacciActionClient()

#     action_client.send_goal(10)

#     rclpy.spin(action_client)


# if __name__ == '__main__':
#     main()


# again running the server and client files
### result output image




##################

### PART 4 - Composing multiple nodes ina single process

# seeing what components are registered and available in the workspace:
ros2 component types

# in the 1st terminal starting the component container
ros2 run rclcpp_components component_container

# in the 2nd terminal verifying that the container is running via ros2:
ros2 component list

# in the 2nd terminal loading the talker component
ros2 component load /ComponentManager composition composition::Talker

# running the listener component
ros2 component load /ComponentManager composition composition::Listener

# inspecting the state of the container
ros2 component list

## Run-time composition using ROS services with a server and client

# in the 1st terminal:
ros2 run rclcpp_components component_container

# in the 2nd terminal
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client

## Compile-time composition using ROS services
ros2 run composition manual_composition

## Run-time composition using dlopen
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so

## Composition using launch actions
ros2 launch composition composition_demo.launch.py

## Unloading components
# in the 1st terminal starting the component container
ros2 run rclcpp_components component_container

# verifying the container is running via ros2
ros2 component list

# in the 2nd terminal
ros2 component load /ComponentManager composition composition::Talker
ros2 component load /ComponentManager composition composition::Listener

# using the unique ID to upload the node from the component container
ros2 component unload /ComponentManager 1 2

## Remapping container name and namespace
ros2 run rclcpp_components component_container --ros-args -r __node:=MyContainer -r __ns:=/ns

# in the 2nd terminal
ros2 component load /ns/MyContainer composition composition::Listener

## Remap component names and namespaces
# in the 1st terminal
ros2 run rclcpp_components component_container

# remap node name
ros2 component load /ComponentManager composition composition::Talker --node-name talker2

# remap namespace
ros2 component load /ComponentManager composition composition::Talker --node-namespace /ns

# remap both
ros2 component load /ComponentManager composition composition::Talker --node-name talker3 --node-namespace /ns2

# checking the component list
ros2 component list

# Passing parameter values into components
ros2 component load /ComponentManager image_tools image_tools::Cam2Image -p burger_mode:=true

# Passing additional arguments into components
ros2 component load /ComponentManager composition composition::Talker -e use_intra_process_comms:=true


### PART 5 - Launch
### Part 5.1 - Creating a launch file

# creating a new directory
mkdir launch

# creating a new file called turtlesim_mimic_launch.py with the following contents:
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='turtlesim',
#             namespace='turtlesim1',
#             executable='turtlesim_node',
#             name='sim'
#         ),
#         Node(
#             package='turtlesim',
#             namespace='turtlesim2',
#             executable='turtlesim_node',
#             name='sim'
#         ),
#         Node(
#             package='turtlesim',
#             executable='mimic',
#             name='mimic',
#             remappings=[
#                 ('/input/pose', '/turtlesim1/turtle1/pose'),
#                 ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
#             ]
#         )
#     ])

# running the file
ros2 launch turtlesim_mimic_launch.py

# seeing the system in action, we opened a new terminal and run the followong command to get the first turtle moving
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"

# getting a better idea of the relationship between the nodes in our launch file
rqt_graph



### Part 5.2 - integrating launch files into ROS 2 packages

# creating a new workspace
mkdir -p launch_ws/src
cd launch_ws/src

ros2 pkg create py_launch_example --build-type ament_python

# Creating the structure to hold launch files
# modifying the setup.py file
# import os
# from glob import glob
# from setuptools import setup

# package_name = 'py_launch_example'

# setup(
#     # Other parameters ...
#     data_files=[
#         # ... Other data files
#         # Include all launch files.
#         (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
#     ]
# )

# creating a file named my_script_launch.py with the following content
# import launch
# import launch_ros.actions

# def generate_launch_description():
#     return launch.LaunchDescription([
#         launch_ros.actions.Node(
#             package='demo_nodes_cpp',
#             executable='talker',
#             name='talker'),
#   ])

# Building and running the launch file
colcon build
ros2 launch



### Part 5.3 - Using substitutions

# creating a package
ros2 pkg create launch_tutorial --build-type ament_python

# creating a directory inside the package
mkdir launch_tutorial/launch

# modifying the setup.py file
# import os
# from glob import glob
# from setuptools import setup

# package_name = 'launch_tutorial'

# setup(
#     # Other parameters ...
#     data_files=[
#         # ... Other data files
#         # Include all launch files.
#         (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
#     ]
# )

# creating a parent launch file named example_main.launch.py with the following content
# from launch_ros.substitutions import FindPackageShare

# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import PathJoinSubstitution, TextSubstitution


# def generate_launch_description():
#     colors = {
#         'background_r': '200'
#     }

#     return LaunchDescription([
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([
#                 PathJoinSubstitution([
#                     FindPackageShare('launch_tutorial'),
#                     'launch',
#                     'example_substitutions.launch.py'
#                 ])
#             ]),
#             launch_arguments={
#                 'turtlesim_ns': 'turtlesim2',
#                 'use_provided_red': 'True',
#                 'new_background_r': TextSubstitution(text=str(colors['background_r']))
#             }.items()
#         )
#     ])

# also creating a substitute file called example_substitutions.launch.py with the following content
# from launch_ros.actions import Node

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
# from launch.conditions import IfCondition
# from launch.substitutions import LaunchConfiguration, PythonExpression


# def generate_launch_description():
#     turtlesim_ns = LaunchConfiguration('turtlesim_ns')
#     use_provided_red = LaunchConfiguration('use_provided_red')
#     new_background_r = LaunchConfiguration('new_background_r')

#     turtlesim_ns_launch_arg = DeclareLaunchArgument(
#         'turtlesim_ns',
#         default_value='turtlesim1'
#     )
#     use_provided_red_launch_arg = DeclareLaunchArgument(
#         'use_provided_red',
#         default_value='False'
#     )
#     new_background_r_launch_arg = DeclareLaunchArgument(
#         'new_background_r',
#         default_value='200'
#     )

#     turtlesim_node = Node(
#         package='turtlesim',
#         namespace=turtlesim_ns,
#         executable='turtlesim_node',
#         name='sim'
#     )
#     spawn_turtle = ExecuteProcess(
#         cmd=[[
#             'ros2 service call ',
#             turtlesim_ns,
#             '/spawn ',
#             'turtlesim/srv/Spawn ',
#             '"{x: 2, y: 2, theta: 0.2}"'
#         ]],
#         shell=True
#     )
#     change_background_r = ExecuteProcess(
#         cmd=[[
#             'ros2 param set ',
#             turtlesim_ns,
#             '/sim background_r ',
#             '120'
#         ]],
#         shell=True
#     )
#     change_background_r_conditioned = ExecuteProcess(
#         condition=IfCondition(
#             PythonExpression([
#                 new_background_r,
#                 ' == 200',
#                 ' and ',
#                 use_provided_red
#             ])
#         ),
#         cmd=[[
#             'ros2 param set ',
#             turtlesim_ns,
#             '/sim background_r ',
#             new_background_r
#         ]],
#         shell=True
#     )

#     return LaunchDescription([
#         turtlesim_ns_launch_arg,
#         use_provided_red_launch_arg,
#         new_background_r_launch_arg,
#         turtlesim_node,
#         spawn_turtle,
#         change_background_r,
#         TimerAction(
#             period=2.0,
#             actions=[change_background_r_conditioned],
#         )
#     ])

# building the package in the workspace root
colcon build

# sourcing the workspace - here I was stuck for a long time - I had to do again 3-4 times
source/install/local_setup.bash && source/install/setup.bash

# launching the file
ros2 launch launch_tutorial example_main.launch.py

# changing the arguments
ros2 launch launch_tutorial example_substitutions.launch.py --show-args

# launching
ros2 launch launch_tutorial example_substitutions.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200




### Part 5.4 - Using event handlers

# creating a new file called example_event_handlers.launch.py in the launch directory of the launch_tutorial package with the following contents:
# from launch_ros.actions import Node

# from launch import LaunchDescription
# from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
#                             LogInfo, RegisterEventHandler, TimerAction)
# from launch.conditions import IfCondition
# from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
#                                 OnProcessIO, OnProcessStart, OnShutdown)
# from launch.events import Shutdown
# from launch.substitutions import (EnvironmentVariable, FindExecutable,
#                                 LaunchConfiguration, LocalSubstitution,
#                                 PythonExpression)


# def generate_launch_description():
#     turtlesim_ns = LaunchConfiguration('turtlesim_ns')
#     use_provided_red = LaunchConfiguration('use_provided_red')
#     new_background_r = LaunchConfiguration('new_background_r')

#     turtlesim_ns_launch_arg = DeclareLaunchArgument(
#         'turtlesim_ns',
#         default_value='turtlesim1'
#     )
#     use_provided_red_launch_arg = DeclareLaunchArgument(
#         'use_provided_red',
#         default_value='False'
#     )
#     new_background_r_launch_arg = DeclareLaunchArgument(
#         'new_background_r',
#         default_value='200'
#     )

#     turtlesim_node = Node(
#         package='turtlesim',
#         namespace=turtlesim_ns,
#         executable='turtlesim_node',
#         name='sim'
#     )
#     spawn_turtle = ExecuteProcess(
#         cmd=[[
#             FindExecutable(name='ros2'),
#             ' service call ',
#             turtlesim_ns,
#             '/spawn ',
#             'turtlesim/srv/Spawn ',
#             '"{x: 2, y: 2, theta: 0.2}"'
#         ]],
#         shell=True
#     )
#     change_background_r = ExecuteProcess(
#         cmd=[[
#             FindExecutable(name='ros2'),
#             ' param set ',
#             turtlesim_ns,
#             '/sim background_r ',
#             '120'
#         ]],
#         shell=True
#     )
#     change_background_r_conditioned = ExecuteProcess(
#         condition=IfCondition(
#             PythonExpression([
#                 new_background_r,
#                 ' == 200',
#                 ' and ',
#                 use_provided_red
#             ])
#         ),
#         cmd=[[
#             FindExecutable(name='ros2'),
#             ' param set ',
#             turtlesim_ns,
#             '/sim background_r ',
#             new_background_r
#         ]],
#         shell=True
#     )

#     return LaunchDescription([
#         turtlesim_ns_launch_arg,
#         use_provided_red_launch_arg,
#         new_background_r_launch_arg,
#         turtlesim_node,
#         RegisterEventHandler(
#             OnProcessStart(
#                 target_action=turtlesim_node,
#                 on_start=[
#                     LogInfo(msg='Turtlesim started, spawning turtle'),
#                     spawn_turtle
#                 ]
#             )
#         ),
#         RegisterEventHandler(
#             OnProcessIO(
#                 target_action=spawn_turtle,
#                 on_stdout=lambda event: LogInfo(
#                     msg='Spawn request says "{}"'.format(
#                         event.text.decode().strip())
#                 )
#             )
#         ),
#         RegisterEventHandler(
#             OnExecutionComplete(
#                 target_action=spawn_turtle,
#                 on_completion=[
#                     LogInfo(msg='Spawn finished'),
#                     change_background_r,
#                     TimerAction(
#                         period=2.0,
#                         actions=[change_background_r_conditioned],
#                     )
#                 ]
#             )
#         ),
#         RegisterEventHandler(
#             OnProcessExit(
#                 target_action=turtlesim_node,
#                 on_exit=[
#                     LogInfo(msg=(EnvironmentVariable(name='USER'),
#                             ' closed the turtlesim window')),
#                     EmitEvent(event=Shutdown(
#                         reason='Window closed'))
#                 ]
#             )
#         ),
#         RegisterEventHandler(
#             OnShutdown(
#                 on_shutdown=[LogInfo(
#                     msg=['Launch was asked to shutdown: ',
#                         LocalSubstitution('event.reason')]
#                 )]
#             )
#         ),
#     ])

# going to the root of the workspace and building the package
colcon build

# sourcing the workspace
source install/local_setup.bash
source install/setup.bash

# launching the file
ros2 launch launch_tutorial example_event_handlers.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200



### Part 5.5 - Managing large projects

# creating launch_turtlesim.launch.py in the launch directory of our launch_tutorial package with the following content
# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource


# def generate_launch_description():
#    turtlesim_world_1 = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([os.path.join(
#          get_package_share_directory('launch_tutorial'), 'launch'),
#          '/turtlesim_world_1.launch.py'])
#       )
#    turtlesim_world_2 = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([os.path.join(
#          get_package_share_directory('launch_tutorial'), 'launch'),
#          '/turtlesim_world_2.launch.py'])
#       )
#    broadcaster_listener_nodes = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([os.path.join(
#          get_package_share_directory('launch_tutorial'), 'launch'),
#          '/broadcaster_listener.launch.py']),
#       launch_arguments={'target_frame': 'carrot1'}.items(),
#       )
#    mimic_node = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([os.path.join(
#          get_package_share_directory('launch_tutorial'), 'launch'),
#          '/mimic.launch.py'])
#       )
#    fixed_frame_node = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([os.path.join(
#          get_package_share_directory('launch_tutorial'), 'launch'),
#          '/fixed_broadcaster.launch.py'])
#       )
#    rviz_node = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([os.path.join(
#          get_package_share_directory('launch_tutorial'), 'launch'),
#          '/turtlesim_rviz.launch.py'])
#       )

#    return LaunchDescription([
#       turtlesim_world_1,
#       turtlesim_world_2,
#       broadcaster_listener_nodes,
#       mimic_node,
#       fixed_frame_node,
#       rviz_node
#    ])

# creating turtlesim_world_1.launch.py with the following content
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, TextSubstitution

# from launch_ros.actions import Node


# def generate_launch_description():
#    background_r_launch_arg = DeclareLaunchArgument(
#       'background_r', default_value=TextSubstitution(text='0')
#    )
#    background_g_launch_arg = DeclareLaunchArgument(
#       'background_g', default_value=TextSubstitution(text='84')
#    )
#    background_b_launch_arg = DeclareLaunchArgument(
#       'background_b', default_value=TextSubstitution(text='122')
#    )

#    return LaunchDescription([
#       background_r_launch_arg,
#       background_g_launch_arg,
#       background_b_launch_arg,
#       Node(
#          package='turtlesim',
#          executable='turtlesim_node',
#          name='sim',
#          parameters=[{
#             'background_r': LaunchConfiguration('background_r'),
#             'background_g': LaunchConfiguration('background_g'),
#             'background_b': LaunchConfiguration('background_b'),
#          }]
#       ),
#    ])


# creating turtlesim_world_2.launch.py with the following content
# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#    config = os.path.join(
#       get_package_share_directory('launch_tutorial'),
#       'config',
#       'turtlesim.yaml'
#       )

#    return LaunchDescription([
#       Node(
#          package='turtlesim',
#          executable='turtlesim_node',
#          namespace='turtlesim2',
#          name='sim',
#          parameters=[config]
#       )
#    ])


# creating turtlesim.yaml file in a new directory called config in our package with the following content
# /turtlesim2/sim:
#    ros__parameters:
#       background_b: 255
#       background_g: 86
#       background_r: 150

# creating turtlesim_world_3.launch.py file with the similar content as world 2 but with one more node

# ...
# Node(
#    package='turtlesim',
#    executable='turtlesim_node',
#    namespace='turtlesim3',
#    name='sim',
#    parameters=[config]
# )

# updating the yaml file
# /**:
#    ros__parameters:
#       background_b: 255
#       background_g: 86
#       background_r: 150

# removing namespace='turtlesim2' line from the turtlesim_world_2.launch.py
# updating the launch_turtlesim.launch.py file
# from launch.actions import GroupAction
# from launch_ros.actions import PushRosNamespace

#    ...
#    turtlesim_world_2 = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([os.path.join(
#          get_package_share_directory('launch_tutorial'), 'launch'),
#          '/turtlesim_world_2.launch.py'])
#       )
#    turtlesim_world_2_with_namespace = GroupAction(
#      actions=[
#          PushRosNamespace('turtlesim2'),
#          turtlesim_world_2,
#       ]
#    )


# replace the turtlesim_world_2 to turtlesim_world_2_with_namespace in the return LaunchDescription statement

# creating a broadcaster_listener.launch.py
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

# from launch_ros.actions import Node


# def generate_launch_description():
#    return LaunchDescription([
#       DeclareLaunchArgument(
#          'target_frame', default_value='turtle1',
#          description='Target frame name.'
#       ),
#       Node(
#          package='turtle_tf2_py',
#          executable='turtle_tf2_broadcaster',
#          name='broadcaster1',
#          parameters=[
#             {'turtlename': 'turtle1'}
#          ]
#       ),
#       Node(
#          package='turtle_tf2_py',
#          executable='turtle_tf2_broadcaster',
#          name='broadcaster2',
#          parameters=[
#             {'turtlename': 'turtle2'}
#          ]
#       ),
#       Node(
#          package='turtle_tf2_py',
#          executable='turtle_tf2_listener',
#          name='listener',
#          parameters=[
#             {'target_frame': LaunchConfiguration('target_frame')}
#          ]
#       ),
#    ])


# Remapping
# creating mimic.launch.py
# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#    return LaunchDescription([
#       Node(
#          package='turtlesim',
#          executable='mimic',
#          name='mimic',
#          remappings=[
#             ('/input/pose', '/turtle2/pose'),
#             ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
#          ]
#       )
#    ])

# creating turtlesim_rviz.launch.py
# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#    rviz_config = os.path.join(
#       get_package_share_directory('turtle_tf2_py'),
#       'rviz',
#       'turtle_rviz.rviz'
#       )

#    return LaunchDescription([
#       Node(
#          package='rviz2',
#          executable='rviz2',
#          name='rviz2',
#          arguments=['-d', rviz_config]
#       )
#    ])


# creating fixed_broadcaster.launch.py
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import EnvironmentVariable, LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description():
#    return LaunchDescription([
#       DeclareLaunchArgument(
#             'node_prefix',
#             default_value=[EnvironmentVariable('USER'), '_'],
#             description='prefix for node name'
#       ),
#       Node(
#             package='turtle_tf2_py',
#             executable='fixed_frame_tf2_broadcaster',
#             name=[LaunchConfiguration('node_prefix'), 'fixed_broadcaster'],
#       ),
#    ])

# updating the setup.py file
# data_files=[
#       ...
#       (os.path.join('share', package_name, 'launch'),
#          glob(os.path.join('launch', '*.launch.py'))),
#       (os.path.join('share', package_name, 'config'),
#          glob(os.path.join('config', '*.yaml'))),
#    ],

# building and running
colcon build
source install/local_setup.bash
source install/setup.bash
ros2 launch launch_tutorial launch_turtlesim.launch.py




### PART 6 - tf2
### Part 6.1 - Introducing tf2

# installing the demo package and its dependencies
sudo apt-get install ros-foxy-turtle-tf2-py ros-foxy-tf2-tools ros-foxy-tf-transformations

# running the demo
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py

# creating a diagram
ros2 run tf2_tools view_frames.py

# looking at the transform of the turtle2 frame with respect to turtle1 frame
ros2 run tf2_ros tf2_echo turtle2 turtle1

# rviz vizualization tool for examining tf2 frames
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz





### Part 6.2 - Writing a static broadcaster (Python)

# creating a new workspace and going to its src directory
mkdir -p ~/learning_tf2/src
cd src

# creating a package inside the workspace
ros2 pkg create --build-type ament_python learning_tf2_py

# Inside the src/learning_tf2_py/learning_tf2_py directory downloading the example static broadcaster code
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/static_turtle_tf2_broadcaster.py

# navigating back to the src/learning_tf2_py and opening package.xml
# modifying the file

# <description>Learning tf2 with rclpy</description>
# <maintainer email="gulizaaitkulova@email.com">gulizaaitkulova</maintainer>
# <license>Apache License 2.0</license>

# and addin the dependencies corresponding to our node's import statements
# <exec_depend>geometry_msgs</exec_depend>
# <exec_depend>python3-numpy</exec_depend>
# <exec_depend>rclpy</exec_depend>
# <exec_depend>tf2_ros_py</exec_depend>
# <exec_depend>turtlesim</exec_depend>

# modifying the setup.py file by adding the following between the console_scripts brackets:
# 'static_turtle_tf2_broadcaster = learning_tf2_py.static_turtle_tf2_broadcaster:main',

# building with rosdep
rosdep install -i --from-path src --rosdistro foxy -y

# I again encountered the same problem as in the previous lab, but even the previous solution didn'r work here which was the following command
rosdep update --include-eol-distros

# building the new package
colcon build --packages-select learning_tf2_py

# in a new terminal
. install/setup.bash

# running
ros2 run learning_tf2_py static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0
ros2 topic echo --qos-reliability reliable --qos-durability transient_local /tf_static

ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll frame_id child_frame_id



### Part 6.3 - Writing a broadcaster in Python

# creating a new workspace called learning_tf2
# downloading broadcaster code

wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/turtle_tf2_broadcaster.py

# modifying the setup.py file for the console_script brackets
'turtle_tf2_broadcaster = learning_tf2_py.turtle_tf2_broadcaster:main',

# creating launch folder with turtle_tf2_demo.py with the following contents
# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='turtlesim',
#             executable='turtlesim_node',
#             name='sim'
#         ),
#         Node(
#             package='learning_tf2_py',
#             executable='turtle_tf2_broadcaster',
#             name='broadcaster1',
#             parameters=[
#                 {'turtlename': 'turtle1'}
#             ]
#         ),
#     ])

# modifying the package.xml
# <exec_depend>launch</exec_depend>
# <exec_depend>launch_ros</exec_depend>

# modifying the setup file
# data_files=[
#     ...
#     (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
# ],

# import os
# from glob import glob


# building
rosdep install -i --from-path src --rosdistro foxy -y

colcon build --packages-select learning_tf2_py

. install/setup.bash

# running
ros2 launch learning_tf2_py turtle_tf2_demo.launch.py

ros2 run turtlesim turtle_teleop_key

ros2 run tf2_ros tf2_echo world turtle1




























