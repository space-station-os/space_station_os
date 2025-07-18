#ifndef SPACE_STATION_ECLSS__CREW_QUARTER_HPP_
#define SPACE_STATION_ECLSS__CREW_QUARTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>

#include "space_station_eclss/action/air_revitalisation.hpp"
#include "space_station_eclss/action/water_recovery.hpp"
#include "space_station_eclss/srv/o2_request.hpp"
#include "space_station_eclss/srv/request_product_water.hpp"

class HumanSimulationNode : public rclcpp::Node
{
public:
  HumanSimulationNode();

private:
  // Parameters
  int crew_size_;
  int events_per_day_;
  int number_of_days_;
  std::string mode_;
  float calorie_intake_;
  float potable_water_intake_;

  // State variables
  int current_day_;
  int events_completed_today_;
  bool ars_sent_ = false;
  bool wrs_sent_ = false;
  bool ogs_sent_ = false;

  double current_o2_reserve_ = 0.0;
  double current_water_reserve_ = 0.0;

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr timer_;

  // Action clients
  rclcpp_action::Client<space_station_eclss::action::AirRevitalisation>::SharedPtr ars_client_;
  rclcpp_action::Client<space_station_eclss::action::WaterRecovery>::SharedPtr wrs_client_;

  // Service clients
  rclcpp::Client<space_station_eclss::srv::O2Request>::SharedPtr o2_service_client_;
  rclcpp::Client<space_station_eclss::srv::RequestProductWater>::SharedPtr water_service_client_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr o2_storage_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr water_reserve_sub_;

  // Simulation loop
  void simulate_event();

  // Helper functions
  void send_ars_goal();
  void send_wrs_goal();
  void request_o2(double o2_required);
  void request_water(float water_required);
};

#endif  // SPACE_STATION_ECLSS__CREW_QUARTER_HPP_
