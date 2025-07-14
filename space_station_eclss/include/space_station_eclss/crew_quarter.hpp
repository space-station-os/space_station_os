#ifndef SPACE_STATION_ECLSS__CREW_QUARTER_HPP_
#define SPACE_STATION_ECLSS__CREW_QUARTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <memory>
#include <future> 
#include "space_station_eclss/action/air_revitalisation.hpp"
#include "space_station_eclss/action/oxygen_generation.hpp"
#include "space_station_eclss/action/water_recovery.hpp"

#include "space_station_eclss/srv/request_product_water.hpp"
#include "space_station_eclss/srv/o2_request.hpp"

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

  int current_day_ = 0;
  int current_event_count_ = 0;

  // Action clients
  rclcpp_action::Client<space_station_eclss::action::AirRevitalisation>::SharedPtr ars_client_;
  rclcpp_action::Client<space_station_eclss::action::OxygenGeneration>::SharedPtr ogs_client_;
  rclcpp_action::Client<space_station_eclss::action::WaterRecovery>::SharedPtr wrs_client_;

  // Service clients
  rclcpp::Client<space_station_eclss::srv::O2Request>::SharedPtr o2_service_client_;
  rclcpp::Client<space_station_eclss::srv::RequestProductWater>::SharedPtr water_service_client_;

  // Timer
  rclcpp::TimerBase::SharedPtr simulation_timer_;

  // Simulation logic
  void simulate_metabolic_cycle();

  // ROS interaction
  void send_o2_request(float o2_amount);
  void send_water_request(float water_amount);
  void send_ars_goal(float co2_mass, float moisture_content);
  void send_ogs_goal(float water_mass);
  void send_wrs_goal(float urine_volume);
};

#endif  // SPACE_STATION_ECLSS__CREW_QUARTER_HPP_
