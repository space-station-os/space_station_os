#ifndef SPACE_STATION_ECLSS__OGS_SYSTEM_HPP_
#define SPACE_STATION_ECLSS__OGS_SYSTEM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "space_station_eclss/action/oxygen_generation.hpp"
#include "space_station_eclss/srv/o2_request.hpp"
#include "space_station_eclss/srv/request_product_water.hpp"
#include "space_station_eclss/srv/co2_request.hpp"
#include "space_station_eclss/srv/grey_water.hpp"
#include <std_msgs/msg/bool.hpp>

class OGSSystem : public rclcpp::Node
{
public:
  OGSSystem();

private:
  // Action server
  rclcpp_action::Server<space_station_eclss::action::OxygenGeneration>::SharedPtr action_server_;
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const space_station_eclss::action::OxygenGeneration::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<space_station_eclss::action::OxygenGeneration>> goal_handle);
  void execute_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<space_station_eclss::action::OxygenGeneration>> goal_handle);

  // Service clients
  rclcpp::Client<space_station_eclss::srv::RequestProductWater>::SharedPtr water_client_;
  rclcpp::Client<space_station_eclss::srv::Co2Request>::SharedPtr co2_client_;
  rclcpp::Client<space_station_eclss::srv::GreyWater>::SharedPtr gray_water_client_;

  // Service server
  rclcpp::Service<space_station_eclss::srv::O2Request>::SharedPtr o2_server_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr o2_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ch4_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr disable_failure_;
  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  bool enable_failure_;
  double electrolysis_temp_;
  double o2_efficiency_;
  double sabatier_efficiency_;
  double sabatier_temp_;
  double sabatier_pressure_;
  double min_o2_capacity_;
  double max_o2_capacity_;

  // Storage
  double latest_o2_ = 0.0;
  double total_ch4_vented_ = 0.0;

  void request_product_water(double amount_liters);
  void request_co2(double co2_mass_kg);
  void send_gray_water(double amount_liters);


  // Service callback
  void o2_service_callback(
    const std::shared_ptr<space_station_eclss::srv::O2Request::Request> request,
    std::shared_ptr<space_station_eclss::srv::O2Request::Response> response);

  // Diagnostics
  void publish_failure_diagnostics(const std::string &unit, const std::string &reason);
  void publish_periodic_status();
};

#endif  // SPACE_STATION_ECLSS__OGS_SYSTEM_HPP_
