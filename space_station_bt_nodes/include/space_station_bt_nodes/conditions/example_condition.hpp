#ifndef SPACE_STATION_BT_NODES__CONDITIONS__EXAMPLE_CONDITION_HPP_
#define SPACE_STATION_BT_NODES__CONDITIONS__EXAMPLE_CONDITION_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "space_station_core/plugin_base.hpp"

namespace space_station_bt_nodes
{
namespace conditions
{

class ExampleCondition : public BT::ConditionNode, public space_station_core::PluginBase
{
public:
  ExampleCondition(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &, const std::string &) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
};

}  // namespace conditions
}  // namespace space_station_bt_nodes

#endif  // SPACE_STATION_BT_NODES__CONDITIONS__EXAMPLE_CONDITION_HPP_
