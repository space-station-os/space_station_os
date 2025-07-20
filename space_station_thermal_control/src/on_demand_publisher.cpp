#include "space_station_thermal_control/on_demand_publisher.hpp"

using std::placeholders::_1;
using std::placeholders::_2;


using namespace space_station_thermal_control;
FocusNodeMonitor::FocusNodeMonitor()
: rclcpp::Node("focus_node_monitor")
{
  node_sub_ = this->create_subscription<space_station_thermal_control::msg::ThermalNodeDataArray>(
    "/thermal/nodes/state", 10,
    std::bind(&FocusNodeMonitor::thermal_node_callback, this, _1));

  service_ = this->create_service<space_station_thermal_control::srv::GetSubTopic>(
    "/thermal/focus_node",
    std::bind(&FocusNodeMonitor::handle_focus_request, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "FocusNodeMonitor node started.");
}

void FocusNodeMonitor::thermal_node_callback(
  const space_station_thermal_control::msg::ThermalNodeDataArray::SharedPtr msg)
{
  latest_nodes_.clear();
  for (const auto &node : msg->nodes) {
    latest_nodes_[node.name] = node;

    // If this node is one of the requested focused nodes, publish it
    if (focused_names_.count(node.name)) {
      auto pub_it = focused_publishers_.find(node.name);
      if (pub_it != focused_publishers_.end()) {
        pub_it->second->publish(node);
      }
    }
  }
}

void FocusNodeMonitor::handle_focus_request(
  const std::shared_ptr<space_station_thermal_control::srv::GetSubTopic::Request> request,
  std::shared_ptr<space_station_thermal_control::srv::GetSubTopic::Response> response)
{
  const std::string &name = request->name;
  auto it = latest_nodes_.find(name);
  if (it == latest_nodes_.end()) {
    response->success = false;
    response->message = "Node '" + name + "' not found.";
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  if (focused_publishers_.find(name) == focused_publishers_.end()) {
    std::string topic = "/thermal/nodes/" + name;
    focused_publishers_[name] = this->create_publisher<space_station_thermal_control::msg::ThermalNodeData>(topic, 10);
    RCLCPP_INFO(this->get_logger(), "Created topic: %s", topic.c_str());
  }

  focused_names_.insert(name);  // Mark this node as to be published continuously

  response->success = true;
  response->message = "Now publishing ongoing data for node '" + name + "' on /thermal/nodes/" + name;
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<space_station_thermal_control::FocusNodeMonitor>());
  rclcpp::shutdown();
  return 0;
}
