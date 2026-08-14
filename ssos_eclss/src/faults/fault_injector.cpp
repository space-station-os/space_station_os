#include "ssos_eclss/faults/fault_injector.hpp"

#include <algorithm>

namespace ssos_eclss
{
namespace faults
{

void FaultInjector::register_fault(const FaultDefinition & def, unsigned seed)
{
  Entry e;
  e.def = def;
  e.active = false;
  if (is_sensor_fault(def.type)) {
    e.sensor = std::make_unique<SensorFault>(def, seed);
  }
  entries_.push_back(std::move(e));
}

void FaultInjector::clear()
{
  entries_.clear();
}

bool FaultInjector::is_active_at(const FaultDefinition & def, double t) const
{
  if (t < def.start_time_s) {
    return false;
  }
  if (def.duration_s < 0.0) {
    return true;  // permanent
  }
  return t <= def.start_time_s + def.duration_s;
}

void FaultInjector::update(double sim_time_s)
{
  current_time_s_ = sim_time_s;
  for (auto & e : entries_) {
    e.active = is_active_at(e.def, sim_time_s);
  }
}

bool FaultInjector::any_active() const
{
  for (const auto & e : entries_) {
    if (e.active) {
      return true;
    }
  }
  return false;
}

std::vector<FaultDefinition> FaultInjector::active_faults() const
{
  std::vector<FaultDefinition> out;
  for (const auto & e : entries_) {
    if (e.active) {
      out.push_back(e.def);
    }
  }
  return out;
}

ActuatorEffect FaultInjector::actuator_effect(const std::string & target) const
{
  ActuatorEffect agg{};
  for (const auto & e : entries_) {
    if (!e.active || e.def.target != target) {
      continue;
    }
    ActuatorFault f(e.def);
    const ActuatorEffect eff = f.effect();
    if (eff.override_position) {
      agg.override_position = true;
      agg.forced_position = eff.forced_position;
    }
    // The most severe (lowest) effectiveness dominates.
    agg.effectiveness = std::min(agg.effectiveness, eff.effectiveness);
  }
  return agg;
}

ThermalEffect FaultInjector::thermal_effect(const std::string & target) const
{
  ThermalEffect agg{};
  for (const auto & e : entries_) {
    if (!e.active || e.def.target != target) {
      continue;
    }
    ThermalFault f(e.def);
    const ThermalEffect eff = f.effect();
    agg.heater_power_factor = std::min(agg.heater_power_factor, eff.heater_power_factor);
    agg.precooler_ua_factor = std::min(agg.precooler_ua_factor, eff.precooler_ua_factor);
    agg.wall_htc_factor = std::max(agg.wall_htc_factor, eff.wall_htc_factor);
  }
  return agg;
}

double FaultInjector::apply_sensor(const std::string & sensor, double true_value)
{
  double value = true_value;
  for (auto & e : entries_) {
    if (!e.active || !e.sensor || e.def.target != sensor) {
      continue;
    }
    value = e.sensor->apply(value, current_time_s_ - e.def.start_time_s);
  }
  return value;
}

double FaultInjector::leak_fault_area(const std::string & target) const
{
  double area = 0.0;
  for (const auto & e : entries_) {
    if (e.active && e.def.type == FaultType::CABIN_LEAK && e.def.target == target) {
      area += e.def.magnitude;  // magnitude is the leak area [m^2]
    }
  }
  return area;
}

}  // namespace faults
}  // namespace ssos_eclss
