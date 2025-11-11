#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "space_station_interfaces/action/coolant.hpp"
#include "space_station_interfaces/srv/load.hpp"
#include <mutex>
namespace space_station_eps
{

class DdcuNode : public rclcpp::Node
{
public:
  explicit DdcuNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // === Subscriptions & Publishers ===
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr primary_voltage_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr output_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temperature_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::Service<space_station_interfaces::srv::Load>::SharedPtr load_srv_;
  // === Action Client for coolant ===
  using Coolant = space_station_interfaces::action::Coolant;
  using GoalHandleCoolant = rclcpp_action::ClientGoalHandle<Coolant>;
  rclcpp_action::Client<Coolant>::SharedPtr coolant_client_;
  std::mutex voltage_mutex_;
  // === Parameters & State ===
  std::string ddcu_type_;
  double nominal_voltage_;
  double regulation_tolerance_;
  double input_voltage_;
  double ddcu_temperature_;

  // === Callbacks ===
  void primaryVoltageCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void publishOutputVoltage(double voltage);
  void callInternalCooling(double heat_j);
  void publishDiagnostics(double input_voltage, double output_voltage);
  void handleLoadRequest(
    const std::shared_ptr<space_station_interfaces::srv::Load::Request> request,
    std::shared_ptr<space_station_interfaces::srv::Load::Response> response);
};

} // namespace space_station_interfaces/eps/
