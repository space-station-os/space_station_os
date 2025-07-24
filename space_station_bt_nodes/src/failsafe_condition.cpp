#include "space_station_bt_nodes/conditions/failsafe_condition.hpp"

namespace space_station_bt_nodes
{
namespace conditions
{

FailsafeCondition::FailsafeCondition(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{}

BT::PortsList FailsafeCondition::providedPorts()
{
  return { BT::InputPort<bool>("failsafe") };
}

BT::NodeStatus FailsafeCondition::tick()
{
  bool fail = false;
  getInput("failsafe", fail);
  return fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

}  // namespace conditions
}  // namespace space_station_bt_nodes

#include <behaviortree_cpp/bt_factory.h>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<space_station_bt_nodes::conditions::FailsafeCondition>("FailsafeCondition");
}

