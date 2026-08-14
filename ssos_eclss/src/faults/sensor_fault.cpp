#include "ssos_eclss/faults/sensor_fault.hpp"

namespace ssos_eclss
{
namespace faults
{

SensorFault::SensorFault(const FaultDefinition & def, unsigned seed)
: def_(def), rng_(seed)
{}

double SensorFault::apply(double true_value, double fault_elapsed_s)
{
  switch (def_.type) {
    case FaultType::SENSOR_STUCK:
      // Latch the first observed value (or the configured magnitude if nonzero).
      if (!latched_) {
        latched_ = true;
        latched_value_ = (def_.magnitude != 0.0) ? def_.magnitude : true_value;
      }
      return latched_value_;

    case FaultType::SENSOR_DRIFT:
      // Linear drift: magnitude is the drift rate [units/s].
      return true_value + def_.magnitude * fault_elapsed_s;

    case FaultType::SENSOR_BIAS:
      // Constant additive offset.
      return true_value + def_.magnitude;

    case FaultType::SENSOR_NOISE:
      // Additive Gaussian noise with std-dev = magnitude.
      return true_value + def_.magnitude * noise_(rng_);

    case FaultType::SENSOR_SCALE:
      // Multiplicative scale error.
      return true_value * def_.magnitude;

    default:
      return true_value;
  }
}

}  // namespace faults
}  // namespace ssos_eclss
