#include <rclcpp/rclcpp.hpp>
#include <pick_and_sort/PickAndSort.hpp>
#include <pick_and_sort_msgs/action/pick_and_sort.hpp>

class PickAndSortActionClient : public rclcpp::Node
{
public:
  PickAndSortActionClient()
    : Node("pick_and_sort_action_client")
  {
    action_client_ = rclcpp_action::create_client<pick_and_sort_msgs::action::PickAndSort>(
        this, "pick_and_sort");

    // Implement your pick and sort logic using the action client
  }

private:
  rclcpp_action::Client<pick_and_sort_msgs::action::PickAndSort>::SharedPtr action_client_;

  // Implement your pick and sort logic using the action client
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickAndSortActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
