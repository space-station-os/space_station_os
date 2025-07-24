#ifndef SPACE_STATION_BT_NODES__CONTROL__EXAMPLE_CONTROL_HPP_
#define SPACE_STATION_BT_NODES__CONTROL__EXAMPLE_CONTROL_HPP_

#include "behaviortree_cpp/control_node.h"
#include "space_station_core/plugin_base.hpp"

namespace space_station_bt_nodes
{
namespace control
{

class ExampleControl : public BT::ControlNode, public space_station_core::PluginBase
{
public:
  ExampleControl(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &, const std::string &) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
};

}  // namespace control
}  // namespace space_station_bt_nodes

#endif  // SPACE_STATION_BT_NODES__CONTROL__EXAMPLE_CONTROL_HPP_
