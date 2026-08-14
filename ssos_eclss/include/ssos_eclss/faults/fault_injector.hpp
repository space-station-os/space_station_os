#ifndef SSOS_ECLSS__FAULTS__FAULT_INJECTOR_HPP_
#define SSOS_ECLSS__FAULTS__FAULT_INJECTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "ssos_eclss/faults/actuator_fault.hpp"
#include "ssos_eclss/faults/fault_types.hpp"
#include "ssos_eclss/faults/sensor_fault.hpp"
#include "ssos_eclss/faults/thermal_fault.hpp"

// Central registry that holds configured faults, tracks which are active given
// the simulation clock, and aggregates their effects so the component models
// can apply them. Faults change real physics behaviour, not just reported
// values. No ROS, no external deps.

namespace ssos_eclss
{
namespace faults
{

class FaultInjector
{
public:
  FaultInjector() = default;

  /// Register a fault definition (with an optional RNG seed for noise).
  void register_fault(const FaultDefinition & def, unsigned seed = 12345u);

  /// Remove all faults.
  void clear();

  /// Update the active set against the current simulation time.
  void update(double sim_time_s);

  /// True if any fault is active.
  bool any_active() const;

  /// Number of registered faults.
  std::size_t count() const { return entries_.size(); }

  /// All currently active fault definitions.
  std::vector<FaultDefinition> active_faults() const;

  /// Aggregate actuator effect for a target component (combines active faults).
  ActuatorEffect actuator_effect(const std::string & target) const;

  /// Aggregate thermal effect for a target component.
  ThermalEffect thermal_effect(const std::string & target) const;

  /// Apply all active sensor faults targeting @p sensor to a true reading.
  double apply_sensor(const std::string & sensor, double true_value);

  /// Total extra leak orifice area [m^2] from active CABIN_LEAK faults.
  double leak_fault_area(const std::string & target = "cabin") const;

private:
  struct Entry
  {
    FaultDefinition def;
    bool active{false};
    std::unique_ptr<SensorFault> sensor;  // only for sensor faults
  };

  bool is_active_at(const FaultDefinition & def, double t) const;

  std::vector<Entry> entries_;
  double current_time_s_{0.0};
};

}  // namespace faults
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__FAULTS__FAULT_INJECTOR_HPP_
