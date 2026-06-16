#include "ssos_eclss/nodes/eclss_diagnostics.hpp"

#include <chrono>
#include <memory>

namespace ssos_eclss
{
namespace nodes
{

SubsystemHeartbeat EclssDiagnostics::make_heartbeat(const rclcpp::Time & stamp,
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

FaultEvent EclssDiagnostics::make_fault(const rclcpp::Time & stamp,
                                        const std::string & subsystem_name,
                                        const std::string & fault_type,
                                        uint8_t severity,
                                        const std::string & description,
                                        const std::vector<std::string> & affected)
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

bool EclssDiagnostics::ars_co2_removal_low(double removal_kg_day, double required_kg_day)
{
  return removal_kg_day < required_kg_day;
}

bool EclssDiagnostics::bed_underheated(double max_bed_temp_k, double target_temp_k)
{
  return max_bed_temp_k < target_temp_k;
}

rclcpp::TimerBase::SharedPtr EclssDiagnostics::maybe_autostart(
  rclcpp_lifecycle::LifecycleNode * node, int delay_ms)
{
  if (!node->has_parameter("autostart")) {
    node->declare_parameter("autostart", false);
  }
  if (!node->get_parameter("autostart").as_bool()) {
    return nullptr;
  }
  // Self-cancelling one-shot: configure -> activate, no ChangeState events.
  auto holder = std::make_shared<rclcpp::TimerBase::SharedPtr>();
  *holder = node->create_wall_timer(
    std::chrono::milliseconds(delay_ms),
    [node, holder]() {
      (*holder)->cancel();
      RCLCPP_INFO(node->get_logger(), "autostart: configuring + activating");
      node->configure();
      node->activate();
    });
  return *holder;
}

}  // namespace nodes
}  // namespace ssos_eclss
