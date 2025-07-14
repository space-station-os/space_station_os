#ifndef SPACE_STATION_ECLSS__CREW_QUARTERS_HPP_
#define SPACE_STATION_ECLSS__CREW_QUARTERS_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "space_station_eclss/action/air_revitalisation.hpp"
#include "space_station_eclss/action/oxygen_generation.hpp"
#include "space_station_eclss/action/water_recovery.hpp"
#include "space_station_eclss/srv/o2_request.hpp"
#include "space_station_eclss/srv/request_product_water.hpp"

class HumanSimulationNode : public rclcpp::Node
{
public:
  HumanSimulationNode();

private:
  void simulate_metabolic_cycle();

  // Timer
  rclcpp::TimerBase::SharedPtr simulation_timer_;

  // Parameters
  int crew_size_;
  int events_per_day_;
  int number_of_days_;
  std::string mode_;
  float calorie_intake_;
  float potable_water_intake_;

  // Simulation Tracking
  int current_event_count_ = 0;
  int current_day_ = 0;

  // Action Clients
  rclcpp_action::Client<space_station_eclss::action::AirRevitalisation>::SharedPtr ars_client_;
  rclcpp_action::Client<space_station_eclss::action::OxygenGeneration>::SharedPtr ogs_client_;
  rclcpp_action::Client<space_station_eclss::action::WaterRecovery>::SharedPtr wrs_client_;
  // Service clients
  rclcpp::Client<space_station_eclss::srv::O2Request>::SharedPtr o2_service_client_;
  rclcpp::Client<space_station_eclss::srv::RequestProductWater>::SharedPtr water_service_client_;

};

#endif  // SPACE_STATION_ECLSS__CREW_QUARTERS_HPP_
