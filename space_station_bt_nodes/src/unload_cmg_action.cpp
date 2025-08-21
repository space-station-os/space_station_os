#include "space_station_bt_nodes/actions/unload_cmg_action.hpp"

namespace space_station_bt_nodes
{
namespace actions
{

UnloadCmgAction::UnloadCmgAction(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  if (!rclcpp::is_initialized()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  node_ = rclcpp::Node::make_shared("unload_cmg_action");
  pub_ = node_->create_publisher<std_msgs::msg::Empty>("gnc/unload_cmg", 10);
}

BT::PortsList UnloadCmgAction::providedPorts()
{
  return {};
}

BT::NodeStatus UnloadCmgAction::tick()
{
  std_msgs::msg::Empty msg;
  pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "Published unload request");
  rclcpp::spin_some(node_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace actions
}  // namespace space_station_bt_nodes

#include <behaviortree_cpp/bt_factory.h>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<space_station_bt_nodes::actions::UnloadCmgAction>("UnloadCMG");
}

