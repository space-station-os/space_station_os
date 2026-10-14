#ifndef SSOS_THERMAL__NODES__THERMAL_DIAGNOSTICS_HPP_
#define SSOS_THERMAL__NODES__THERMAL_DIAGNOSTICS_HPP_

#include <string>
#include <vector>

#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"

// Heartbeat/fault helpers shared by every ssos_thermal node, mirroring
// ssos_eclss::nodes::EclssDiagnostics. Originally sized for a single caller
// (subsystem_name baked in as "thermal"); parameterized once a second node
// (CoolantNode, subsystem_name "coolant") needed its own identity -- same
// growth EclssDiagnostics went through serving five subsystems.

namespace ssos_thermal
{
namespace nodes
{

using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using FaultEvent = space_station_interfaces::msg::FaultEvent;

class ThermalDiagnostics
{
public:
  /// Build a heartbeat message.
  static SubsystemHeartbeat make_heartbeat(
    const rclcpp::Time & stamp, const std::string & subsystem_name,
    uint8_t lifecycle_state, bool healthy, const std::string & status_message);

  /// Build a fault-event message.
  static FaultEvent make_fault(
    const rclcpp::Time & stamp, const std::string & subsystem_name,
    const std::string & fault_type, uint8_t severity,
    const std::string & description,
    const std::vector<std::string> & affected_interfaces = {});

  /// Edge-detector: call every tick with the current health. Returns true
  /// exactly once per healthy->unhealthy transition, so a fault publishes
  /// once instead of spamming the fault bus every tick.
  bool should_raise_fault(bool healthy);

  /// If the node has (or is given) an "autostart" parameter set true, return
  /// a one-shot timer that configures then activates the node shortly after
  /// startup, so a launch file doesn't need to emit lifecycle ChangeState
  /// events. Returns nullptr when autostart is false. The caller must keep
  /// the returned timer alive (store it in a member).
  static rclcpp::TimerBase::SharedPtr maybe_autostart(
    rclcpp_lifecycle::LifecycleNode * node, int delay_ms = 300);

private:
  bool was_healthy_ = true;
};

}  // namespace nodes
}  // namespace ssos_thermal

#endif  // SSOS_THERMAL__NODES__THERMAL_DIAGNOSTICS_HPP_
