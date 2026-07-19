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
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/msg/world_state.hpp"
#include "space_station_interfaces/srv/register_subsystem.hpp"

#include "ssos_eclss/ars/four_bed_system.hpp"
#include "ssos_eclss/nodes/eclss_parameter_bridge.hpp"

// Lifecycle node wrapping the FourBedSystem ARS physics. Reads cabin conditions
// from /sim/world_state (Epic A boundary), advances the 4BMS each step, and
// publishes telemetry, CO2 removal rate, a heartbeat and fault events.

namespace ssos_eclss
{
namespace nodes
{

using WorldState = space_station_interfaces::msg::WorldState;
using RegisterSubsystem = space_station_interfaces::srv::RegisterSubsystem;
using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using FaultEvent = space_station_interfaces::msg::FaultEvent;
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
  void on_world_state(const WorldState::SharedPtr msg);
  void on_cabin_co2(const std_msgs::msg::Float64::SharedPtr msg);
  void register_with_manager();
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  // Physics
  std::unique_ptr<ars::FourBedSystem> ars_;
  std::unique_ptr<EclssParameterBridge> bridge_;
  ars::CabinConditions cabin_{};
  ars::ArsResult last_result_{};
  bool have_world_state_{false};
  // Closed-loop: live cabin CO2 (ppm) from cabin_node, preferred over the
  // simulator's world-state CO2 so removal tracks the actual cabin and the
  // cabin<->ARS loop self-regulates to a steady ppCO2.
  bool have_cabin_co2_{false};

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr co2_removal_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    telemetry_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    bed_states_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    cycle_phase_pub_;
  rclcpp_lifecycle::LifecyclePublisher<SubsystemHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp_lifecycle::LifecyclePublisher<FaultEvent>::SharedPtr fault_pub_;

  // Subscriber
  rclcpp::Subscription<WorldState>::SharedPtr world_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cabin_co2_sub_;

  // Service client
  rclcpp::Client<RegisterSubsystem>::SharedPtr register_client_;

  // Timers
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;

  // Parameter callback handle
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Config
  double step_rate_hz_{1.0};
  double co2_required_kg_day_{4.16};
  rclcpp::Time last_step_time_;
  bool first_step_{true};
  bool params_dirty_{false};
  bool was_healthy_{true};  // edge-trigger for the CO2-removal fault
  // Auto-faults off by default: in the closed loop, removal settles at the crew
  // production rate (== the requirement), so a threshold check there is a
  // knife-edge. Faults are injected explicitly (sim fault-injection YAML).
  bool enable_auto_faults_{false};
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__ARS_NODE_HPP_
