#ifndef SSOS_GNC__NODES__GNC_DIAGNOSTICS_HPP_
#define SSOS_GNC__NODES__GNC_DIAGNOSTICS_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"

#include "ssos_gnc/flight/attitude_control_system.hpp"

namespace ssos_gnc
{

namespace nodes
{

using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using FaultEvent = space_station_interfaces::msg::FaultEvent;
using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;

DiagnosticArray build_diagnostics(
  const rclcpp::Time & stamp,
  const std::string & subsystem_name,
  const flight::ControlResult & result);

SubsystemHeartbeat build_heartbeat(
  const rclcpp::Time & stamp,
  const std::string & subsystem_name,
  uint8_t lifecycle_state,
  bool healthy,
  const std::string & status_message);

FaultEvent build_fault_event(
  const rclcpp::Time & stamp,
  const std::string & subsystem_name,
  const flight::faults::FaultDefinition & fault,
  bool activated);
}  // namespace nodes
}  // namespace ssos_gnc

#endif  // SSOS_GNC__NODES__GNC_DIAGNOSTICS_HPP_
