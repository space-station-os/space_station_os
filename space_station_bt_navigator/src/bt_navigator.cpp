#include "space_station_bt_navigator/bt_navigator.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <fstream>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace space_station_bt_navigator
{

BtNavigator::BtNavigator(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("bt_navigator", options)
{
  this->declare_parameter<bool>("failsafe", false);
}

space_station_bt::BehaviorTreeEngine::Status BtNavigator::execute()
{
  std::string tree_file;
  if (!get_parameter("tree_xml", tree_file)) {
    tree_file = "main_mission_tree.xml";
  }

  std::string tree_path = tree_file;
  
  std::ifstream file(tree_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("BtNavigator"), 
                 "Failed to open behavior tree file: %s", tree_path.c_str());
    return space_station_bt::BehaviorTreeEngine::Status::FAILURE;
  }

  std::string xml_content((std::istreambuf_iterator<char>(file)),
                          std::istreambuf_iterator<char>());
  file.close();

  auto blackboard = BT::Blackboard::create();
  blackboard->set("failsafe", failsafe_);

  try {
    auto tree = bt_engine_->createTreeFromText(xml_content, blackboard);
    return bt_engine_->run(&tree);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("BtNavigator"), 
                 "Failed to create or run behavior tree: %s", ex.what());
    return space_station_bt::BehaviorTreeEngine::Status::FAILURE;
  }
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
