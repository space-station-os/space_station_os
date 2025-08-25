#ifndef SPACE_STATION_BT__BEHAVIOR_TREE_ENGINE_HPP_
#define SPACE_STATION_BT__BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace space_station_bt
{

class BehaviorTreeEngine
{
public:
  explicit BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries);
  ~BehaviorTreeEngine();

  enum class Status { SUCCESS, FAILURE, CANCELED };

  Status run(BT::Tree * tree);
  BT::Tree createTreeFromText(const std::string & xml,
                              BT::Blackboard::Ptr blackboard);

private:
  std::vector<std::string> plugin_libraries_;
  BT::BehaviorTreeFactory factory_;
};

}  // namespace space_station_bt

#endif  // SPACE_STATION_BT__BEHAVIOR_TREE_ENGINE_HPP_
