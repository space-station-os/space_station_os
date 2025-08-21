#ifndef SPACE_STATION_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define SPACE_STATION_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

#include <memory>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "space_station_bt/behavior_tree_engine.hpp"
#include "space_station_utils/utils.hpp"

namespace space_station_bt_navigator
{

class BtNavigator : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit BtNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &);

  space_station_bt::BehaviorTreeEngine::Status execute();
  std::shared_ptr<space_station_bt::BehaviorTreeEngine> bt_engine_;
  bool failsafe_{false};
};

}  // namespace space_station_bt_navigator

#endif  // SPACE_STATION_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
