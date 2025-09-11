// cooling.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <space_station_eclss/srv/request_product_water.hpp>
#include <space_station_thermal_control/action/coolant.hpp>
#include <space_station_thermal_control/srv/vent_heat.hpp>
#include <space_station_thermal_control/msg/external_loop_status.hpp>

namespace space_station_thermal_control
{

class CoolantActionServer : public rclcpp::Node
{
public:
  using Coolant = space_station_thermal_control::action::Coolant;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Coolant>;

  explicit CoolantActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // Parameters
  double mass_kg_;
  double cp_water_;
  double transfer_efficiency_;
  double vent_threshold_kj_;
  bool diagnostics_enabled_;
  int goal_counter_ = 0;
  double current_temp_c_ = 25.0;
  double lasted_vented_heat_ = 0.0;

  // ROS interfaces
  rclcpp_action::Server<Coolant>::SharedPtr action_server_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr internal_loop_pub_;
  rclcpp::Publisher<space_station_thermal_control::msg::ExternalLoopStatus>::SharedPtr external_loop_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;

  rclcpp::Client<space_station_eclss::srv::RequestProductWater>::SharedPtr product_water_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr grey_water_client_;
  rclcpp::Service<space_station_thermal_control::srv::VentHeat>::SharedPtr venting_server_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Internal Methods
  void simulateHeatRise();
  void recycleWater();
  void publishInternalLoop();
  void publishExternalLoop(double received_heat_kj);
  void publishDiagnostics(bool status, const std::string &message);

  void handleVenting(const std::shared_ptr<space_station_thermal_control::srv::VentHeat::Request> req,
                     const std::shared_ptr<space_station_thermal_control::srv::VentHeat::Response> res);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid,
                                         std::shared_ptr<const Coolant::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);
};

} // namespace space_station_thermal_control
