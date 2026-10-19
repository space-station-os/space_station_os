#ifndef SSOS_EPS__SOLAR__SSU_MODEL_HPP_
#define SSOS_EPS__SOLAR__SSU_MODEL_HPP_

#include "ssos_eps/solar/solar_parameters.hpp"

// Sequential shunt unit. Regulates the array output onto the primary bus at a
// fixed setpoint and reports the power and current it can deliver. Whatever
// the downstream equipment does not draw is shunted back at the array. No ROS.

namespace ssos_eps
{
namespace solar
{

struct SsuOutput
{
  bool regulating;             // false when array input is below dropout
  double bus_voltage_v;        // regulated output voltage [V]
  double available_power_w;    // power deliverable to the bus [W]
  double available_current_a;  // current deliverable at bus_voltage_v [A]
};

class SsuModel
{
public:
  explicit SsuModel(const SsuParams & params = default_ssu_params());

  void reset();

  /// Regulate the array output for one timestep.
  /// @param dt             timestep [s]
  /// @param array_power_w  DC power from the array [W]
  const SsuOutput & step(double dt, double array_power_w);

  void set_params(const SsuParams & params) {params_ = params;}
  const SsuParams & params() const {return params_;}
  const SsuOutput & output() const {return output_;}

private:
  SsuParams params_;
  SsuOutput output_{};
  double elapsed_s_{0.0};
};

}  // namespace solar
}  // namespace ssos_eps

#endif  // SSOS_EPS__SOLAR__SSU_MODEL_HPP_
