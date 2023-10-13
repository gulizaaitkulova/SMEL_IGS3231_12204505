#!/bin/bash

### 12204505 Guliza Aitkulova
### Smart Mobility Engineering Lab - Week 6 lab - ROS tutorials
### Advanced - Part 4 - Recording a bag from a node (C++)

# installing rosbag2 packages
sudo apt install ros-foxy-rosbag2

cd ros2_ws

# creating a new package
ros2 pkg create --build-type ament_cmake bag_recorder_nodes --dependencies rclcpp rosbag2_cpp example_interfaces

# updating the package.xml file
# <description>C++ bag writing tutorial</description>
# <maintainer email="you@email.com">Your Name</maintainer>
# <license>Apache License 2.0</license>

# writing the C++ node
cd ~/ros2_ws/src/bag_recorder_nodes/src

touch simple_bag_recorder.cpp

echo "#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

using std::placeholders::_1;

class SimpleBagRecorder : public rclcpp::Node
{
public:
  SimpleBagRecorder()
  : Node("simple_bag_recorder")
  {
    const rosbag2_cpp::StorageOptions storage_options({"my_bag", "sqlite3"});
    const rosbag2_cpp::ConverterOptions converter_options(
      {rmw_get_serialization_format(),
       rmw_get_serialization_format()});
    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

    writer_->open(storage_options, converter_options);

    writer_->create_topic(
      {"chatter",
       "std_msgs/msg/String",
       rmw_get_serialization_format(),
       ""});

    subscription_ = create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
  }

private:
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      new rcutils_uint8_array_t,
      [this](rcutils_uint8_array_t *msg) {
        auto fini_return = rcutils_uint8_array_fini(msg);
        delete msg;
        if (fini_return != RCUTILS_RET_OK) {
          RCLCPP_ERROR(get_logger(),
            "Failed to destroy serialized message %s", rcutils_get_error_string().str);
        }
      });
    *bag_message->serialized_data = msg->release_rcl_serialized_message();

    bag_message->topic_name = "chatter";
    if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error getting current time: %s",
        rcutils_get_error_string().str);
    }

    writer_->write(bag_message);
  }

  rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
}" > simple_bag_recorder.cpp

# adding executable to the CMakeLists.txt file
cd ..
echo "add_executable(simple_bag_recorder src/simple_bag_recorder.cpp)
ament_target_dependencies(simple_bag_recorder rclcpp rosbag2_cpp)

install(TARGETS
  simple_bag_recorder
  DESTINATION lib/${PROJECT_NAME}
)" > CMakeLists.txt

cd ~/ros2_ws

# building the package
colcon build --packages-select bag_recorder_nodes

#sourcing the setup files
source install/setup.bash

# running
ros2 run bag_recorder_nodes simple_bag_recorder
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
ros2 bag play my_bag

### Writing another node
cd ~/ros2_ws/src/bag_recorder_nodes/src

touch data_generator_node.cpp

echo "#include <chrono>

#include <example_interfaces/msg/int32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

using namespace std::chrono_literals;

class DataGenerator : public rclcpp::Node
{
public:
  DataGenerator()
  : Node("data_generator")
  {
    data.data = 0;
    const rosbag2_cpp::StorageOptions storage_options({"timed_synthetic_bag", "sqlite3"});
    const rosbag2_cpp::ConverterOptions converter_options(
      {rmw_get_serialization_format(),
       rmw_get_serialization_format()});
    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

    writer_->open(storage_options, converter_options);

    writer_->create_topic(
      {"synthetic",
       "example_interfaces/msg/Int32",
       rmw_get_serialization_format(),
       ""});

    timer_ = create_wall_timer(1s, std::bind(&DataGenerator::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto serializer = rclcpp::Serialization<example_interfaces::msg::Int32>();
    auto serialized_message = rclcpp::SerializedMessage();
    serializer.serialize_message(&data, &serialized_message);

    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      new rcutils_uint8_array_t,
      [this](rcutils_uint8_array_t *msg) {
        auto fini_return = rcutils_uint8_array_fini(msg);
        delete msg;
        if (fini_return != RCUTILS_RET_OK) {
          RCLCPP_ERROR(get_logger(),
            "Failed to destroy serialized message %s", rcutils_get_error_string().str);
        }
      });
    *bag_message->serialized_data = serialized_message.release_rcl_serialized_message();

    bag_message->topic_name = "synthetic";
    if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error getting current time: %s",
        rcutils_get_error_string().str);
    }

    writer_->write(bag_message);
    ++data.data;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
  example_interfaces::msg::Int32 data;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataGenerator>());
  rclcpp::shutdown();
  return 0;
}" > data_generator_node.cpp

# adding executables
cd ..
echo "add_executable(data_generator_node src/data_generator_node.cpp)
ament_target_dependencies(data_generator_node rclcpp rosbag2_cpp example_interfaces)

install(TARGETS
  data_generator_node
  DESTINATION lib/${PROJECT_NAME}
)" > CMakeLists.txt

# building and running

# building the package
colcon build --packages-select bag_recorder_nodes

#sourcing the setup files
source install/setup.bash

ros2 run bag_recorder_nodes data_generator_node
ros2 bag play timed_synthetic_bag
ros2 topic echo /synthetic

# writing another node
cd ~/ros2_ws/src/bag_recorder_nodes/src

touch data_generator_executable.cpp

echo "#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <example_interfaces/msg/int32.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

int main(int, char**)
{
  example_interfaces::msg::Int32 data;
  data.data = 0;
  const rosbag2_cpp::StorageOptions storage_options({"big_synthetic_bag", "sqlite3"});
  const rosbag2_cpp::ConverterOptions converter_options(
    {rmw_get_serialization_format(),
     rmw_get_serialization_format()});
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_ =
    std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

  writer_->open(storage_options, converter_options);

  writer_->create_topic(
    {"synthetic",
     "example_interfaces/msg/Int32",
     rmw_get_serialization_format(),
     ""});

  rcutils_time_point_value_t time_stamp;
  if (rcutils_system_time_now(&time_stamp) != RCUTILS_RET_OK) {
    std::cerr << "Error getting current time: " <<
      rcutils_get_error_string().str;
    return 1;
  }
  for (int32_t ii = 0; ii < 100; ++ii) {
    auto serializer = rclcpp::Serialization<example_interfaces::msg::Int32>();
    auto serialized_message = rclcpp::SerializedMessage();
    serializer.serialize_message(&data, &serialized_message);

    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      new rcutils_uint8_array_t,
      [](rcutils_uint8_array_t *msg) {
        auto fini_return = rcutils_uint8_array_fini(msg);
        delete msg;
        if (fini_return != RCUTILS_RET_OK) {
          std::cerr << "Failed to destroy serialized message " <<
            rcutils_get_error_string().str;
        }
      });
    *bag_message->serialized_data = serialized_message.release_rcl_serialized_message();

    bag_message->topic_name = "synthetic";
    bag_message->time_stamp = time_stamp;

    writer_->write(bag_message);
    ++data.data;
    time_stamp += 1000000000;
  }

  return 0;
}" > data_generator_executable.cpp


# adding executables

cd ..
echo "add_executable(data_generator_executable src/data_generator_executable.cpp)
ament_target_dependencies(data_generator_executable rclcpp rosbag2_cpp example_interfaces)

install(TARGETS
  data_generator_executable
  DESTINATION lib/${PROJECT_NAME}
)" > CMakeLists.txt

# building the package
colcon build --packages-select bag_recorder_nodes

#sourcing the setup files
source install/setup.bash

# running

ros2 run bag_recorder_nodes data_generator_executable
ros2 bag play big_synthetic_bag
ros2 topic echo /synthetic