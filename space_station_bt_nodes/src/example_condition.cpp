#include "space_station_bt_nodes/conditions/example_condition.hpp"

namespace space_station_bt_nodes
{
namespace conditions
{

ExampleCondition::ExampleCondition(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{}

BT::PortsList ExampleCondition::providedPorts()
{
  return {};
}

BT::NodeStatus ExampleCondition::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace conditions
}  // namespace space_station_bt_nodes

#include <behaviortree_cpp/bt_factory.h>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<space_station_bt_nodes::conditions::ExampleCondition>("ExampleCondition");
}
