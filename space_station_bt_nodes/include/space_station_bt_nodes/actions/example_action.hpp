#ifndef SPACE_STATION_BT_NODES__ACTIONS__EXAMPLE_ACTION_HPP_
#define SPACE_STATION_BT_NODES__ACTIONS__EXAMPLE_ACTION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "space_station_core/plugin_base.hpp"

namespace space_station_bt_nodes
{
namespace actions
{

class ExampleAction : public BT::SyncActionNode, public space_station_core::PluginBase
{
public:
  ExampleAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

  // PluginBase overrides
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &, const std::string &) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
};

}  // namespace actions
}  // namespace space_station_bt_nodes

#endif  // SPACE_STATION_BT_NODES__ACTIONS__EXAMPLE_ACTION_HPP_
