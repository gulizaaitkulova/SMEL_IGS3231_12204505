#include <rclcpp/rclcpp.hpp>
#include <pick_and_sort/PickAndSort.hpp>

class PickAndSortActionServer : public rclcpp::Node
{
public:
  PickAndSortActionServer()
    : Node("pick_and_sort_action_server")
  {
    action_server_ = rclcpp_action::create_server<pick_and_sort::action::PickAndSort>(
        this, "pick_and_sort",
        std::bind(&PickAndSortActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PickAndSortActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&PickAndSortActionServer::handle_accepted, this, std::placeholders::_1));

    // Initialize the necessary components for your pick and sort action server
  }

private:
  rclcpp_action::Server<pick_and_sort::action::PickAndSort>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const pick_and_sort::action::PickAndSort::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with id '%s'", uuid.c_str());

    // Check if the goal is valid and return the appropriate response
    // You can also perform any necessary initialization here

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<pick_and_sort::action::PickAndSort>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

    // Check if the goal can be canceled and return the appropriate response

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<pick_and_sort::action::PickAndSort>> goal_handle)
  {
    // Start executing the pick and sort action
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickAndSortActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
