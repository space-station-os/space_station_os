#include "ssos_eps/nodes/eps_diagnostics.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>

namespace ssos_eps
{
namespace nodes
{

SubsystemHeartbeat EpsDiagnostics::make_heartbeat(const rclcpp::Time & stamp,
                                                  const std::string & subsystem_name,
                                                  uint8_t lifecycle_state, bool healthy,
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

FaultEvent EpsDiagnostics::make_fault(const rclcpp::Time & stamp,
                                      const std::string & subsystem_name,
                                      const std::string & fault_type, uint8_t severity,
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

BCDUStatus EpsDiagnostics::make_bcdu_status(const rclcpp::Time & stamp,
                                            const std::string & frame_id,
                                            const bcdu::BcduState & state)
{
  BCDUStatus msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.mode = bcdu::to_string(state.mode);
  msg.bus_voltage = state.bus_voltage_v;
  msg.regulation_voltage = state.regulation_voltage_v;
  msg.current_draw = state.current_draw_a;
  msg.fault = state.faulted || state.degraded;
  msg.fault_message = state.status_message == "nominal" ? "" : state.status_message;
  return msg;
}

BatteryState EpsDiagnostics::make_battery_state(const rclcpp::Time & stamp,
                                                const std::string & id,
                                                const battery::BatteryOruState & state,
                                                const battery::BatteryOruParams & params,
                                                double capacity_ah)
{
  BatteryState b;
  b.header.stamp = stamp;
  b.header.frame_id = id;
  b.voltage = static_cast<float>(state.terminal_voltage_v);
  b.temperature = static_cast<float>(state.temperature_c);
  // BatteryState uses positive current for charging.
  b.current = static_cast<float>(-state.current_a);
  b.charge = static_cast<float>(state.soc * capacity_ah);
  b.capacity = static_cast<float>(capacity_ah);
  b.design_capacity = static_cast<float>(params.capacity_ah);
  b.percentage = static_cast<float>(state.soc);

  switch (state.mode) {
    case battery::BatteryMode::CHARGING:
      b.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_CHARGING;
      break;
    case battery::BatteryMode::DISCHARGING:
      b.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
      break;
    case battery::BatteryMode::IDLE:
      b.power_supply_status = state.soc >= 1.0 ?
        BatteryState::POWER_SUPPLY_STATUS_FULL :
        BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
      break;
  }
  switch (state.health) {
    case battery::BatteryHealth::GOOD:
      b.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_GOOD;
      break;
    case battery::BatteryHealth::OVERHEAT:
      b.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
      break;
    case battery::BatteryHealth::FAILED:
      b.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_DEAD;
      break;
  }
  b.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  b.present = true;
  b.serial_number = id;

  const auto n = static_cast<std::size_t>(std::max(params.n_series_cells, 1));
  b.cell_voltage.assign(n, static_cast<float>(state.terminal_voltage_v / n));
  b.cell_temperature.assign(n, static_cast<float>(state.temperature_c));
  return b;
}

void EpsDiagnostics::add_value(diagnostic_msgs::msg::DiagnosticStatus & status,
                               const std::string & key, double value, int precision)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(precision) << value;
  add_value(status, key, ss.str());
}

void EpsDiagnostics::add_value(diagnostic_msgs::msg::DiagnosticStatus & status,
                               const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue pair;
  pair.key = key;
  pair.value = value;
  status.values.push_back(pair);
}

rclcpp::TimerBase::SharedPtr EpsDiagnostics::maybe_autostart(
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
}  // namespace ssos_eps
