#include "space_station_bt_nodes/decorators/example_decorator.hpp"

namespace space_station_bt_nodes
{
namespace decorators
{

ExampleDecorator::ExampleDecorator(const std::string & name, const BT::NodeConfig & config)
: BT::DecoratorNode(name, config)
{}

BT::PortsList ExampleDecorator::providedPorts()
{
  return {};
}

BT::NodeStatus ExampleDecorator::tick()
{
  return child_node_->executeTick();
}

}  // namespace decorators
}  // namespace space_station_bt_nodes

#include <behaviortree_cpp/bt_factory.h>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<space_station_bt_nodes::decorators::ExampleDecorator>("ExampleDecorator");
}
