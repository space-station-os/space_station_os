#include "ssos_eclss/ogs/o2_controller.hpp"

#include <algorithm>

namespace ssos_eclss
{
namespace ogs
{

O2PressureController::O2PressureController(const O2ControllerParams & params)
: params_(params)
{
  reset(params_.min_current_a);
}

void O2PressureController::reset(double current_a)
{
  integral_a_ = std::clamp(current_a, params_.min_current_a, params_.max_current_a);
  current_ = integral_a_;
}

double O2PressureController::update(double dt, double measured_kpa)
{
  const double error = params_.setpoint_kpa - measured_kpa;
  if (dt > 0.0 && params_.integral_time_s > 0.0) {
    integral_a_ += params_.kp_a_per_kpa * error * dt / params_.integral_time_s;
    integral_a_ = std::clamp(integral_a_, params_.min_current_a, params_.max_current_a);
  }
  current_ = std::clamp(integral_a_ + params_.kp_a_per_kpa * error,
      params_.min_current_a, params_.max_current_a);
  return current_;
}

O2ControllerParams default_o2_controller_params()
{
  // A 1 A change moves O2 by about 0.2 kg/day; in a 388 m^3 cabin that shifts
  // ppO2 by about 0.04 kPa/day, so these gains settle over roughly a day.
  O2ControllerParams p{};
  p.setpoint_kpa = 21.0;
  p.kp_a_per_kpa = 20.0;
  p.integral_time_s = 86400.0;
  p.min_current_a = 0.0;
  p.max_current_a = 46.9;     // ISS OGA maximum process current
  return p;
}

}  // namespace ogs
}  // namespace ssos_eclss
