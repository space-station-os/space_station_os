#ifndef SSOS_ECLSS__FAULTS__ACTUATOR_FAULT_HPP_
#define SSOS_ECLSS__FAULTS__ACTUATOR_FAULT_HPP_

#include "ssos_eclss/faults/fault_types.hpp"

// Actuator fault models. These corrupt the actual physics: valves stuck,
// blower degradation/failure, pump failure. No ROS, no external deps.

namespace ssos_eclss
{
namespace faults
{

/// Effect of an actuator fault on a commanded actuator.
struct ActuatorEffect
{
  bool override_position{false};  // valve: force position
  double forced_position{0.0};    // [0,1]
  double effectiveness{1.0};      // multiplier on flow/throughput [0,1]
};

/// Applies a single actuator fault.
class ActuatorFault
{
public:
  explicit ActuatorFault(const FaultDefinition & def);

  /// Effect on the targeted actuator (independent of time once active).
  ActuatorEffect effect() const;

  const FaultDefinition & definition() const { return def_; }

private:
  FaultDefinition def_;
};

}  // namespace faults
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__FAULTS__ACTUATOR_FAULT_HPP_
