#include "space_station_bt_nodes/actions/example_action.hpp"

namespace space_station_bt_nodes
{
namespace actions
{

ExampleAction::ExampleAction(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList ExampleAction::providedPorts()
{
  return {};
}

BT::NodeStatus ExampleAction::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace actions
}  // namespace space_station_bt_nodes

#include <behaviortree_cpp/bt_factory.h>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<space_station_bt_nodes::actions::ExampleAction>("ExampleAction");
}
