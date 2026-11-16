#ifndef SSOS_EPS__NODES__EPS_DIAGNOSTICS_HPP_
#define SSOS_EPS__NODES__EPS_DIAGNOSTICS_HPP_

#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "space_station_interfaces/msg/bcdu_status.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"

#include "ssos_eps/battery/battery_oru_model.hpp"
#include "ssos_eps/bcdu/bcdu_model.hpp"

// Helpers for building heartbeat, fault-event and telemetry messages from the
// EPS physics state.

namespace ssos_eps
{
namespace nodes
{

using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using FaultEvent = space_station_interfaces::msg::FaultEvent;
using BCDUStatus = space_station_interfaces::msg::BCDUStatus;
using BatteryState = sensor_msgs::msg::BatteryState;

class EpsDiagnostics
{
public:
  static SubsystemHeartbeat make_heartbeat(const rclcpp::Time & stamp,
                                           const std::string & subsystem_name,
                                           uint8_t lifecycle_state, bool healthy,
                                           const std::string & status_message);

  static FaultEvent make_fault(const rclcpp::Time & stamp,
                               const std::string & subsystem_name,
                               const std::string & fault_type, uint8_t severity,
                               const std::string & description,
                               const std::vector<std::string> & affected_interfaces = {});

  static BCDUStatus make_bcdu_status(const rclcpp::Time & stamp,
                                     const std::string & frame_id,
                                     const bcdu::BcduState & state);

  static BatteryState make_battery_state(const rclcpp::Time & stamp,
                                         const std::string & id,
                                         const battery::BatteryOruState & state,
                                         const battery::BatteryOruParams & params,
                                         double capacity_ah);

  /// Append a key/value pair, formatted with a fixed number of decimals.
  static void add_value(diagnostic_msgs::msg::DiagnosticStatus & status,
                        const std::string & key, double value, int precision = 3);
  static void add_value(diagnostic_msgs::msg::DiagnosticStatus & status,
                        const std::string & key, const std::string & value);

  /// If the node's "autostart" parameter is true, return a one-shot timer that
  /// configures then activates the node after autostart_delay_ms. The caller
  /// keeps the returned timer alive. Returns nullptr when autostart is false.
  static rclcpp::TimerBase::SharedPtr maybe_autostart(
    rclcpp_lifecycle::LifecycleNode * node, int delay_ms = 300);
};

}  // namespace nodes
}  // namespace ssos_eps

#endif  // SSOS_EPS__NODES__EPS_DIAGNOSTICS_HPP_
