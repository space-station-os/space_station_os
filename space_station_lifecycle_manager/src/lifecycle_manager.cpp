#include "space_station_lifecycle_manager/lifecycle_manager.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

namespace space_station_lifecycle_manager
{

LifecycleManager::LifecycleManager(const rclcpp::NodeOptions & options)
: rclcpp::Node("lifecycle_manager", options)
{
  declare_parameter<std::vector<std::string>>("managed_nodes", std::vector<std::string>{});
  get_parameter("managed_nodes", managed_nodes_);
}

bool LifecycleManager::changeState(const std::string & node_name, uint8_t transition)
{
  auto client = create_client<lifecycle_msgs::srv::ChangeState>(node_name + "/change_state");
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(get_logger(), "Service not available for %s", node_name.c_str());
    return false;
  }
  auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  req->transition.id = transition;
  auto result = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(shared_from_this(), result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Failed to change state for %s", node_name.c_str());
    return false;
  }
  return result.get()->success;
}

bool LifecycleManager::startup()
{
  for (const auto & node : managed_nodes_) {
    if (!changeState(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
      return false;
    }
    if (!changeState(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return false;
    }
  }
  return true;
}

bool LifecycleManager::shutdown()
{
  for (const auto & node : managed_nodes_) {
    changeState(node, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    changeState(node, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  }
  return true;
}

}  // namespace space_station_lifecycle_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(space_station_lifecycle_manager::LifecycleManager)
