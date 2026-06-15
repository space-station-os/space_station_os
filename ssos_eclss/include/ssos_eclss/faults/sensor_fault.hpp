#ifndef SSOS_ECLSS__FAULTS__SENSOR_FAULT_HPP_
#define SSOS_ECLSS__FAULTS__SENSOR_FAULT_HPP_

#include <random>

#include "ssos_eclss/faults/fault_types.hpp"

// Sensor fault models. These corrupt a reported reading; the underlying physics
// is unaffected. Stuck-at, drift, bias, additive noise and scale error.
// No ROS, no external deps.

namespace ssos_eclss
{
namespace faults
{

/// Applies a single sensor fault to a stream of readings.
class SensorFault
{
public:
  explicit SensorFault(const FaultDefinition & def, unsigned seed = 12345u);

  /// Transform a true reading into the faulted reading at elapsed fault time.
  /// @param true_value  the physically correct value
  /// @param fault_elapsed_s seconds since the fault activated
  double apply(double true_value, double fault_elapsed_s);

  const FaultDefinition & definition() const { return def_; }

private:
  FaultDefinition def_;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_{0.0, 1.0};
  bool latched_{false};
  double latched_value_{0.0};
};

}  // namespace faults
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__FAULTS__SENSOR_FAULT_HPP_
