#ifndef SPACE_STATION_BT_NODES__DECORATORS__EXAMPLE_DECORATOR_HPP_
#define SPACE_STATION_BT_NODES__DECORATORS__EXAMPLE_DECORATOR_HPP_

#include "behaviortree_cpp/decorator_node.h"
#include "space_station_core/plugin_base.hpp"

namespace space_station_bt_nodes
{
namespace decorators
{

class ExampleDecorator : public BT::DecoratorNode, public space_station_core::PluginBase
{
public:
  ExampleDecorator(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &, const std::string &) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
};

}  // namespace decorators
}  // namespace space_station_bt_nodes

#endif  // SPACE_STATION_BT_NODES__DECORATORS__EXAMPLE_DECORATOR_HPP_
