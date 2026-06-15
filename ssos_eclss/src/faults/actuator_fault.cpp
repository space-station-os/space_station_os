#include "ssos_eclss/faults/actuator_fault.hpp"

#include <algorithm>

namespace ssos_eclss
{
namespace faults
{

ActuatorFault::ActuatorFault(const FaultDefinition & def) : def_(def) {}

ActuatorEffect ActuatorFault::effect() const
{
  ActuatorEffect e{};
  switch (def_.type) {
    case FaultType::VALVE_STUCK_OPEN:
      e.override_position = true;
      e.forced_position = 1.0;
      break;
    case FaultType::VALVE_STUCK_CLOSED:
      e.override_position = true;
      e.forced_position = 0.0;
      e.effectiveness = 0.0;
      break;
    case FaultType::BLOWER_DEGRADED:
      // magnitude is the residual effectiveness fraction (e.g. 0.6).
      e.effectiveness = std::clamp(def_.magnitude, 0.0, 1.0);
      break;
    case FaultType::BLOWER_FAILED:
    case FaultType::PUMP_FAILED:
      e.effectiveness = 0.0;
      break;
    default:
      break;
  }
  return e;
}

}  // namespace faults
}  // namespace ssos_eclss
