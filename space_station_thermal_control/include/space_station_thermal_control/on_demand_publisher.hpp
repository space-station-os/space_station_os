#pragma once

#include <rclcpp/rclcpp.hpp>
#include <space_station_interfaces/msg/thermal_node_data_array.hpp>
#include <space_station_interfaces/msg/thermal_node_data.hpp>
#include <space_station_interfaces/srv/get_sub_topic.hpp>

#include <string>
#include <unordered_map>
#include <unordered_set>

namespace space_station_thermal_control{
class FocusNodeMonitor : public rclcpp::Node
{
public:
  FocusNodeMonitor();

private:
  void thermal_node_callback(const space_station_interfaces::msg::ThermalNodeDataArray::SharedPtr msg);
  void handle_focus_request(
    const std::shared_ptr<space_station_interfaces::srv::GetSubTopic::Request> request,
    std::shared_ptr<space_station_interfaces::srv::GetSubTopic::Response> response);

  rclcpp::Subscription<space_station_interfaces::msg::ThermalNodeDataArray>::SharedPtr node_sub_;
  rclcpp::Service<space_station_interfaces::srv::GetSubTopic>::SharedPtr service_;

  std::unordered_map<std::string, space_station_interfaces::msg::ThermalNodeData> latest_nodes_;
  std::unordered_map<std::string, rclcpp::Publisher<space_station_interfaces::msg::ThermalNodeData>::SharedPtr> focused_publishers_;
  std::unordered_set<std::string> focused_names_;
};
}
