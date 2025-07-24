#include "space_station_bt_nodes/control/example_control.hpp"

namespace space_station_bt_nodes
{
namespace control
{

ExampleControl::ExampleControl(const std::string & name, const BT::NodeConfig & config)
: BT::ControlNode(name, config)
{}

BT::PortsList ExampleControl::providedPorts()
{
  return {};
}

BT::NodeStatus ExampleControl::tick()
{
  for (auto & child : children_nodes_) {
    const auto status = child->executeTick();
    if (status != BT::NodeStatus::FAILURE) {
      return status;
    }
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace control
}  // namespace space_station_bt_nodes

#include <behaviortree_cpp/bt_factory.h>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<space_station_bt_nodes::control::ExampleControl>("ExampleControl");
}
