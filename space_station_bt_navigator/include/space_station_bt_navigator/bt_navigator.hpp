#ifndef SPACE_STATION_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define SPACE_STATION_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "space_station_bt/behavior_tree_engine.hpp"
#include "space_station_utils/utils.hpp"

namespace space_station_bt_navigator
{

class BtNavigator : public rclcpp::Node
{
public:
  explicit BtNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  space_station_bt::BehaviorTreeEngine::Status execute();
  space_station_bt::BehaviorTreeEngine bt_engine_;
};

}  // namespace space_station_bt_navigator

#endif  // SPACE_STATION_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
