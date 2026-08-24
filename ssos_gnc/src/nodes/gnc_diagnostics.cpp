#include "ssos_gnc/nodes/gnc_diagnostics.hpp"

#include <string>

namespace ssos_gnc
{

namespace nodes
{

namespace
{
diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}  // namespace nodes

diagnostic_msgs::msg::KeyValue kv(const std::string & key, double value)
{
  return kv(key, std::to_string(value));
}  // namespace ssos_gnc
}

DiagnosticArray build_diagnostics(
  const rclcpp::Time & stamp,
  const std::string & subsystem_name,
  const flight::ControlResult & result)
{
  DiagnosticArray array;
  array.header.stamp = stamp;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = subsystem_name + "/attitude_control";
  status.hardware_id = subsystem_name;

  if (!result.healthy) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "fault active";
  } else if (result.saturated) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "actuator saturated";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "nominal";
  }

  status.values.push_back(kv("mode", flight::ActuationModeMachine::mode_name(result.mode)));
  status.values.push_back(kv("attitude_error_deg", common::rad_to_deg(result.attitude_error_rad)));
  status.values.push_back(kv("cmg_momentum_frac", result.cmg_momentum_frac));
  status.values.push_back(kv("manipulability", result.manipulability));
  status.values.push_back(kv("unload_active", result.unload_active ? "true" : "false"));
  status.values.push_back(kv("dead_zone_active", result.dead_zone_active ? "true" : "false"));
  status.values.push_back(kv("estimator_coasting", result.estimator_coasting ? "true" : "false"));
  status.values.push_back(kv("saturated", result.saturated ? "true" : "false"));

  array.status.push_back(status);
  return array;
}

SubsystemHeartbeat build_heartbeat(
  const rclcpp::Time & stamp,
  const std::string & subsystem_name,
  uint8_t lifecycle_state,
  bool healthy,
  const std::string & status_message)
{
  SubsystemHeartbeat hb;
  hb.stamp = stamp;
  hb.subsystem_name = subsystem_name;
  hb.lifecycle_state = lifecycle_state;
  hb.healthy = healthy;
  hb.status_message = status_message;
  return hb;
}

FaultEvent build_fault_event(
  const rclcpp::Time & stamp,
  const std::string & subsystem_name,
  const flight::faults::FaultDefinition & fault,
  bool activated)
{
  FaultEvent event;
  event.stamp = stamp;
  event.subsystem_name = subsystem_name;
  event.fault_type = flight::faults::fault_type_name(fault.type);

  event.severity = static_cast<uint8_t>(fault.severity);

  event.description = activated ?
    ("activated: " + fault.description) :
    ("cleared: " + fault.description);
  return event;
}
}
}
