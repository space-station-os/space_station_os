#ifndef SSOS_ECLSS__NODES__ECLSS_DIAGNOSTICS_HPP_
#define SSOS_ECLSS__NODES__ECLSS_DIAGNOSTICS_HPP_

#include <string>
#include <vector>

#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"

// Shared helpers for building heartbeat and fault-event messages and for
// node autostart. Used by all subsystem nodes.

namespace ssos_eclss
{
namespace nodes
{

using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using FaultEvent = space_station_interfaces::msg::FaultEvent;

class EclssDiagnostics
{
public:
  /// Build a heartbeat message.
  static SubsystemHeartbeat make_heartbeat(const rclcpp::Time & stamp,
                                           const std::string & subsystem_name,
                                           uint8_t lifecycle_state, bool healthy,
                                           const std::string & status_message);

  /// Build a fault-event message.
  static FaultEvent make_fault(const rclcpp::Time & stamp,
                               const std::string & subsystem_name,
                               const std::string & fault_type, uint8_t severity,
                               const std::string & description,
                               const std::vector<std::string> & affected_interfaces = {});

  /// If the node has (or is given) an "autostart" parameter set true, return a
  /// one-shot timer that configures then activates the node shortly after
  /// startup. This lets a launch file bring the node fully up without emitting
  /// any lifecycle ChangeState events (which are racy/redundant when several
  /// managed nodes start together). Returns nullptr when autostart is false.
  /// The caller must keep the returned timer alive (store it in a member).
  static rclcpp::TimerBase::SharedPtr maybe_autostart(
    rclcpp_lifecycle::LifecycleNode * node, int delay_ms = 300);
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__ECLSS_DIAGNOSTICS_HPP_
