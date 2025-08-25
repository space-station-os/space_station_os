#include "space_station_bt/behavior_tree_engine.hpp"

namespace space_station_bt
{

BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries)
: plugin_libraries_(plugin_libraries)
{
  for (const auto & plugin : plugin_libraries_) {
    try {
      factory_.registerFromPlugin(plugin);
    } catch (const std::exception & ex) {
      RCLCPP_WARN(rclcpp::get_logger("BehaviorTreeEngine"),
        "Failed to load plugin %s: %s", plugin.c_str(), ex.what());
    }
  }
}

BehaviorTreeEngine::~BehaviorTreeEngine() = default;

BehaviorTreeEngine::Status BehaviorTreeEngine::run(BT::Tree * tree)
{
  BT::NodeStatus status = tree->tickRootWhileRunning(std::chrono::milliseconds(100));
  
  if (status == BT::NodeStatus::SUCCESS) {
    return Status::SUCCESS;
  }
  return Status::FAILURE;
}

BT::Tree BehaviorTreeEngine::createTreeFromText(const std::string & xml,
                                                BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromText(xml, blackboard);
}

}  // namespace space_station_bt
