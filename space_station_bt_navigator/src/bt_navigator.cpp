#include "space_station_bt_navigator/bt_navigator.hpp"
#include "behaviortree_cpp/bt_factory.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace space_station_bt_navigator
{

BtNavigator::BtNavigator(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("bt_navigator", options)
{
  // space_station_utils::get_parameter_or(this, "failsafe", false);

  this->declare_parameter<bool>("failsafe", false);
}

space_station_bt::BehaviorTreeEngine::Status BtNavigator::execute()
{
  static const char * xml =
    R"(<root main_tree_to_execute='MainTree'>
         <BehaviorTree ID='MainTree'>
           <Fallback name='root'>
             <Sequence name='nominal'>
               <FailsafeCondition failsafe='{failsafe}'/>
               <ExampleAction name='DoWork'/>
             </Sequence>
             <UnloadCMG name='Unload'/>
           </Fallback>
         </BehaviorTree>
       </root>)";

  auto blackboard = BT::Blackboard::create();

  blackboard->set("failsafe", failsafe_);

  auto tree = bt_engine_.createTreeFromText(xml, blackboard);
  return bt_engine_.run(&tree);
}

CallbackReturn BtNavigator::on_configure(const rclcpp_lifecycle::State &)
{
  bt_engine_ = std::make_shared<space_station_bt::BehaviorTreeEngine>(std::vector<std::string>{"space_station_bt_nodes"});
  get_parameter("failsafe", failsafe_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn BtNavigator::on_activate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn BtNavigator::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn BtNavigator::on_cleanup(const rclcpp_lifecycle::State &)
{
  bt_engine_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn BtNavigator::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

}  // namespace space_station_bt_navigator

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(space_station_bt_navigator::BtNavigator)
