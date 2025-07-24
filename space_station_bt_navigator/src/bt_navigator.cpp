#include "space_station_bt_navigator/bt_navigator.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace space_station_bt_navigator
{

BtNavigator::BtNavigator(const rclcpp::NodeOptions & options)
: rclcpp::Node("bt_navigator", options),
  bt_engine_({"space_station_bt_nodes"})
{
  space_station_utils::get_parameter_or(this, "failsafe", false);
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
  bool fs = space_station_utils::get_parameter_or(this, "failsafe", false);
  blackboard->set("failsafe", fs);
  auto tree = bt_engine_.createTreeFromText(xml, blackboard);
  return bt_engine_.run(&tree);
}

}  // namespace space_station_bt_navigator

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(space_station_bt_navigator::BtNavigator)
