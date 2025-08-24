#ifndef SPACE_STATION_THERMAL_CONTROL__EXTERNAL_LOOP_HPP_
#define SPACE_STATION_THERMAL_CONTROL__EXTERNAL_LOOP_HPP_

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "space_station_thermal_control/srv/coolant_flow.hpp"
#include "space_station_thermal_control/srv/vent_heat.hpp"

#include "space_station_thermal_control/msg/internal_loop_status.hpp"
#include "space_station_thermal_control/msg/external_loop_status.hpp"

namespace space_station_thermal_control
{

class ExternalLoopA : public rclcpp::Node
{
public:
  explicit ExternalLoopA();

private:
  // --- State variables ---
  double ammonia_temp_{0.0};
  bool ammonia_filled_{false};
  bool awaiting_ammonia_response_{false};

  // --- ROS interfaces ---
  rclcpp::Subscription<space_station_thermal_control::msg::InternalLoopStatus>::SharedPtr internal_sub_;
  rclcpp::Client<space_station_thermal_control::srv::CoolantFlow>::SharedPtr ammonia_client_;
  rclcpp::Client<space_station_thermal_control::srv::VentHeat>::SharedPtr radiator_client_;
  rclcpp::Publisher<space_station_thermal_control::msg::ExternalLoopStatus>::SharedPtr loop_status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;

  // --- Parameters ---
  double request_volume_kg_{5.0};         // ammonia mass requested per fill
  double loop_a_threshold_c_{45.0};       // minimum Loop A temp to engage heat exchange
  double assumed_received_heat_kj_{1200.0}; // modeled heat received per exchange (kJ)
  double cp_ammonia_kj_per_kg_c_{4.7};    // ammonia Cp (kJ/(kg·°C))

  // --- Core methods ---
  void try_ammonia_refill();
  void interface_heat_exchanger(
    const space_station_thermal_control::msg::InternalLoopStatus::SharedPtr msg);

  // --- Diagnostics helper ---
  void publish_diag(int8_t level, const std::string & message);
};

}  // namespace space_station_thermal_control

#endif  // SPACE_STATION_THERMAL_CONTROL__EXTERNAL_LOOP_HPP_
