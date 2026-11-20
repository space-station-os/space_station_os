#ifndef SSOS_ECLSS__NODES__ARS_NODE_HPP_
#define SSOS_ECLSS__NODES__ARS_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "space_station_interfaces/msg/cabin_atmosphere.hpp"

#include "ssos_eclss/nodes/eps_power_client.hpp"
#include "ssos_eclss/ars/four_bed_system.hpp"
#include "ssos_eclss/nodes/eclss_parameter_bridge.hpp"
#include "ssos_eclss/nodes/node_support.hpp"

// Lifecycle node wrapping the FourBedSystem ARS physics. Takes the cabin
// atmosphere from cabin_node, advances the 4BMS each step, and publishes the
// CO2 removal rate, bed telemetry and health.

namespace ssos_eclss
{
namespace nodes
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ArsNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ArsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /// Last computed CO2 removal rate [kg/day] (for testing).
  double last_co2_removal_kg_day() const { return last_result_.co2_removal_rate_kg_day; }

private:
  void step();
  void on_cabin(const space_station_interfaces::msg::CabinAtmosphere::SharedPtr msg);
  void apply_parameters();
  void apply_saturation();
  void update_saturation(double dt);
  double capture_efficiency() const;
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  // Physics
  std::unique_ptr<ars::FourBedSystem> ars_;
  std::unique_ptr<EclssParameterBridge> bridge_;
  ars::CabinConditions cabin_{};
  ars::ArsResult last_result_{};

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr co2_removal_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    telemetry_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    bed_states_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    cycle_phase_pub_;
  rclcpp::Subscription<space_station_interfaces::msg::CabinAtmosphere>::SharedPtr cabin_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;

  // Parameter callback handle
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Config
  double step_rate_hz_{1.0};
  // Health from performance: single-pass capture efficiency (CO2 removed over
  // CO2 drawn in) averaged over about a half-cycle, and the process flow.
  double min_capture_efficiency_{0.5};
  double capture_time_constant_s_{4800.0};
  double capture_ema_{0.0};
  // Without bed heaters the beds stop regenerating: the saturated fraction
  // rises over a half-cycle and decays again once the heaters are back.
  ars::ArsParameters active_params_{};
  double saturation_{0.0};
  double design_flow_scfm_{26.0};
  double warm_start_s_{0.0};          // pre-roll before going live
  bool params_dirty_{false};
  bool enable_auto_faults_{true};

  StepClock clock_{120.0};   // bounds the bed sub-steps one tick can take
  InjectedFaults drills_;
  HealthReporter health_;

  // Power from ssos_eps; full_power_w_ is the demand at full function.
  EpsPowerClient power_{this, "eclss_ars", 0};
  bool eps_coupling_{true};
  double full_power_w_{0.0};
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__ARS_NODE_HPP_
