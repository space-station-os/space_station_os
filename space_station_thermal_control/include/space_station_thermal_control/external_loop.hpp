#ifndef space_station_thermal_control__EXTERNAL_LOOP_HPP_
#define space_station_thermal_control__EXTERNAL_LOOP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "space_station_thermal_control/srv/coolant_flow.hpp"
#include "space_station_thermal_control/msg/internal_loop_status.hpp"
#include "space_station_thermal_control/msg/external_loop_status.hpp"
#include "space_station_thermal_control/srv/vent_heat.hpp"

namespace space_station_thermal_control
{

class ExternalLoopA : public rclcpp::Node
{
public:
  ExternalLoopA();

private:
  // State variables
  double ammonia_temp_;
  bool ammonia_filled_;
  bool awaiting_ammonia_response_;

  // ROS interfaces
  rclcpp::Client<space_station_thermal_control::srv::CoolantFlow>::SharedPtr ammonia_client_;
  rclcpp::Publisher<space_station_thermal_control::msg::ExternalLoopStatus>::SharedPtr loop_status_pub_;
  rclcpp::Subscription<space_station_thermal_control::msg::InternalLoopStatus>::SharedPtr internal_sub_;
  rclcpp::Client<space_station_thermal_control::srv::VentHeat>::SharedPtr radiator_client_;

  rclcpp::TimerBase::SharedPtr retry_timer_;

  // Core methods
  void try_ammonia_refill();
  void interface_heat_exchanger(const space_station_thermal_control::msg::InternalLoopStatus::SharedPtr msg);
};

}  // namespace space_station_thermal_control

#endif  // space_station_thermal_control__EXTERNAL_LOOP_HPP_
