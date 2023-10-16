#ifndef PICK_AND_SORT_ACTION__PICK_AND_SORT_HPP_
#define PICK_AND_SORT_ACTION__PICK_AND_SORT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <pick_and_sort_action/action/pick_and_sort.hpp>

class PickAndSortAction : public rclcpp::Node
{
public:
  using GoalHandlePickAndSort = rclcpp_action::ServerGoalHandle<pick_and_sort_action::action::PickAndSort>;

  explicit PickAndSortAction(const rclcpp::NodeOptions& options);

private:
  rclcpp_action::Server<pick_and_sort_action::action::PickAndSort>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const pick_and_sort_action::action::PickAndSort::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePickAndSort> goal_handle);

  void execute(const std::shared_ptr<GoalHandlePickAndSort> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandlePickAndSort> goal_handle);

  void publish_feedback(const std::shared_ptr<GoalHandlePickAndSort> goal_handle, const std::string& feedback);

  void publish_result(const std::shared_ptr<GoalHandlePickAndSort> goal_handle, int32_t result);
};

#endif  // PICK_AND_SORT_ACTION__PICK_AND_SORT_HPP_
