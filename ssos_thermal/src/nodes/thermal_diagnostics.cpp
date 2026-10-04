#include "ssos_thermal/nodes/thermal_diagnostics.hpp"

#include <chrono>
#include <memory>

namespace ssos_thermal
{
namespace nodes
{

SubsystemHeartbeat ThermalDiagnostics::make_heartbeat(
  const rclcpp::Time & stamp, const std::string & subsystem_name,
  uint8_t lifecycle_state, bool healthy, const std::string & status_message)
{
  SubsystemHeartbeat hb;
  hb.stamp = stamp;
  hb.subsystem_name = subsystem_name;
  hb.lifecycle_state = lifecycle_state;
  hb.healthy = healthy;
  hb.status_message = status_message;
  return hb;
}

FaultEvent ThermalDiagnostics::make_fault(
  const rclcpp::Time & stamp, const std::string & subsystem_name,
  const std::string & fault_type, uint8_t severity,
  const std::string & description, const std::vector<std::string> & affected)
{
  FaultEvent ev;
  ev.stamp = stamp;
  ev.subsystem_name = subsystem_name;
  ev.fault_type = fault_type;
  ev.severity = severity;
  ev.description = description;
  ev.affected_interfaces = affected;
  return ev;
}

bool ThermalDiagnostics::should_raise_fault(bool healthy)
{
  const bool raise = !healthy && was_healthy_;
  was_healthy_ = healthy;
  return raise;
}

rclcpp::TimerBase::SharedPtr ThermalDiagnostics::maybe_autostart(
  rclcpp_lifecycle::LifecycleNode * node, int delay_ms)
{
  if (!node->has_parameter("autostart")) {
    node->declare_parameter("autostart", false);
  }
  if (!node->has_parameter("autostart_delay_ms")) {
    node->declare_parameter("autostart_delay_ms", delay_ms);
  }
  if (!node->get_parameter("autostart").as_bool()) {
    return nullptr;
  }
  const int delay = static_cast<int>(node->get_parameter("autostart_delay_ms").as_int());
  // Self-cancelling one-shot: configure -> activate, no ChangeState events.
  auto holder = std::make_shared<rclcpp::TimerBase::SharedPtr>();
  *holder = node->create_wall_timer(
    std::chrono::milliseconds(delay),
    [node, holder]() {
      (*holder)->cancel();
      RCLCPP_INFO(node->get_logger(), "autostart: configuring + activating");
      node->configure();
      node->activate();
    });
  return *holder;
}

}  // namespace nodes
}  // namespace ssos_thermal
