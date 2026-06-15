#ifndef SSOS_ECLSS__FAULTS__THERMAL_FAULT_HPP_
#define SSOS_ECLSS__FAULTS__THERMAL_FAULT_HPP_

#include "ssos_eclss/faults/fault_types.hpp"

// Thermal fault models: heater element failure (partial/total), precooler
// degradation and insulation loss. These corrupt the physics. No ROS.

namespace ssos_eclss
{
namespace faults
{

/// Effect of a thermal fault on thermal model coefficients.
struct ThermalEffect
{
  double heater_power_factor{1.0};   // multiplier on heater power [0,1+]
  double precooler_ua_factor{1.0};   // multiplier on precooler UA [0,1]
  double wall_htc_factor{1.0};       // multiplier on wall loss coefficient [>=1]
};

/// Applies a single thermal fault.
class ThermalFault
{
public:
  explicit ThermalFault(const FaultDefinition & def);

  ThermalEffect effect() const;

  const FaultDefinition & definition() const { return def_; }

private:
  FaultDefinition def_;
};

}  // namespace faults
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__FAULTS__THERMAL_FAULT_HPP_
