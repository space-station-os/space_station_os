#include "ssos_eclss/faults/thermal_fault.hpp"

#include <algorithm>

namespace ssos_eclss
{
namespace faults
{

ThermalFault::ThermalFault(const FaultDefinition & def) : def_(def) {}

ThermalEffect ThermalFault::effect() const
{
  ThermalEffect e{};
  switch (def_.type) {
    case FaultType::HEATER_PARTIAL:
      // magnitude is the surviving power fraction (e.g. 7/19 elements).
      e.heater_power_factor = std::clamp(def_.magnitude, 0.0, 1.0);
      break;
    case FaultType::HEATER_FAILED:
      e.heater_power_factor = 0.0;
      break;
    case FaultType::PRECOOLER_DEGRADED:
      // magnitude is the residual UA fraction.
      e.precooler_ua_factor = std::clamp(def_.magnitude, 0.0, 1.0);
      break;
    case FaultType::INSULATION_LOSS:
      // magnitude is the wall-loss amplification (>= 1).
      e.wall_htc_factor = std::max(1.0, def_.magnitude);
      break;
    default:
      break;
  }
  return e;
}

}  // namespace faults
}  // namespace ssos_eclss
