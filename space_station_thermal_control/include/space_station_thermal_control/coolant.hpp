#ifndef SPACE_STATION_THERMAL_CONTROL__COOLANT_HPP_
#define SPACE_STATION_THERMAL_CONTROL__COOLANT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "space_station_eclss/srv/request_product_water.hpp"

#include "space_station_thermal_control/msg/internal_loop_status.hpp"
#include "space_station_thermal_control/msg/external_loop_status.hpp"
#include "space_station_thermal_control/msg/tank_status.hpp"

#include "space_station_thermal_control/srv/coolant_flow.hpp"
#include "space_station_thermal_control/srv/internal_loop.hpp"
#include "space_station_thermal_control/srv/node_heat_flow.hpp"
#include "space_station_eclss/srv/request_product_water.hpp"
#include "space_station_thermal_control/msg/tank_status.hpp"
#include "space_station_thermal_control/msg/internal_loop_status.hpp"
#include "space_station_thermal_control/msg/external_loop_status.hpp"
#include "std_msgs/msg/bool.hpp"

#include <chrono>
#include <future>
#include <random>
#include <string>

namespace space_station_thermal_control
{

class CoolantManager : public rclcpp::Node
{
public:
  explicit CoolantManager();

private:
  // ---- Callbacks / operations ----
  void handle_fill_loops(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);


  void request_water();  // uses default param volume

  void apply_heat_reduction(
    const space_station_thermal_control::msg::ExternalLoopStatus::SharedPtr msg);

  void handle_ammonia(
    const std::shared_ptr<space_station_thermal_control::srv::CoolantFlow::Request> request,
    std::shared_ptr<space_station_thermal_control::srv::CoolantFlow::Response> response);

  void handle_thermal_state_request(
    const std::shared_ptr<space_station_thermal_control::srv::InternalLoop::Request> request,
    std::shared_ptr<space_station_thermal_control::srv::InternalLoop::Response> response);

  void handle_heatflow(
    const std::shared_ptr<space_station_thermal_control::srv::NodeHeatFlow::Request> request,
    std::shared_ptr<space_station_thermal_control::srv::NodeHeatFlow::Response> response);

  void publish_loop_temperatures();
  void control_cycle();

  // ---- Helpers ----
  void publish_diag(int8_t level, const std::string & message);
  double clamp_internal_temp(double t) const;
  void update_ammonia_pressure_and_safety();

  // ---- Publishers / Subscribers / Services ----
  rclcpp::Publisher<space_station_thermal_control::msg::InternalLoopStatus>::SharedPtr loop_temp_pub_;
  rclcpp::Subscription<space_station_thermal_control::msg::ExternalLoopStatus>::SharedPtr loop_temp_sub_;
  rclcpp::Publisher<space_station_thermal_control::msg::TankStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heat_available_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr grey_water_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heater_signal_pub_;
  rclcpp::Client<space_station_eclss::srv::RequestProductWater>::SharedPtr water_client_;

  rclcpp::Service<space_station_thermal_control::srv::CoolantFlow>::SharedPtr ammonia_server_;
  rclcpp::Service<space_station_thermal_control::srv::InternalLoop>::SharedPtr thermal_state_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fill_loops_server_;
  rclcpp::Service<space_station_thermal_control::srv::NodeHeatFlow>::SharedPtr heatflow_server_;


  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // ---- State ----
  double tank_capacity_;        // [L]
  double ammonia_volume_;       // [L]
  double ammonia_temp_;         // [°C]
  double ammonia_pressure_;     // [Pa]
  bool heater_on_;
  bool heater_logged_;

  double loop_mass_kg_;         // [kg] water mass in internal loop
  double initial_temperature_;  // [°C]
  double current_temperature_;  // [°C]

  int control_step_counter_;
  bool water_acquired_;
  bool water_request_pending_;
  bool publish_timer_started_ {false};

  std::shared_future<space_station_eclss::srv::RequestProductWater::Response::SharedPtr> water_future_;

  // ---- Parameters (runtime) ----
  double cp_j_per_kg_c_;             // [J/(kg·°C)]
  double min_internal_temp_c_;       // [°C]
  double max_internal_temp_c_;       // [°C]
  double control_period_s_;          // [s]
  double publish_period_s_;          // [s]
  double water_request_liters_;      // [L]
  int    refresh_period_cycles_;     // [cycles]
  double refresh_volume_l_;          // [L]
  int    recycle_every_goals_;       // [goals]
  double recycle_volume_l_;          // [L]
  std::string heatflow_input_mode_;  // "temp_c" or "joules"
  double heat_boost_gain_;           // multiplicative factor on applied heat

  // Ammonia / pressure params
  double pressure_base_pa_;
  double pressure_gain_pa_per_c_;
  double pressure_ref_offset_c_;   // add to temp before gain (matches old +20)
  double max_ammonia_pressure_pa_;
  double heater_on_below_c_;
  double heater_off_above_c_;
  double heater_warm_rate_c_per_cycle_;

  // Fault injection
  bool enable_failures_;
  double p_drop_water_req_;
  double p_drop_ammonia_grant_;

  // Counters
  int successful_goals_ {0};

  // RNG
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uni01_{0.0, 1.0};
};

}  // namespace space_station_thermal_control

#endif  // SPACE_STATION_THERMAL_CONTROL__COOLANT_HPP_
