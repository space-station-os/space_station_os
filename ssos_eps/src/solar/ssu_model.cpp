#include "ssos_eps/solar/ssu_model.hpp"

#include <algorithm>
#include <cmath>

#include "ssos_eps/common/units.hpp"

namespace ssos_eps
{
namespace solar
{

SsuModel::SsuModel(const SsuParams & params)
: params_(params)
{
  reset();
}

void SsuModel::reset()
{
  output_ = SsuOutput{};
  elapsed_s_ = 0.0;
}

const SsuOutput & SsuModel::step(double dt, double array_power_w)
{
  elapsed_s_ += std::max(dt, 0.0);

  if (array_power_w <= params_.min_input_power_w) {
    output_ = SsuOutput{};
    return output_;
  }

  double ripple = 0.0;
  if (params_.ripple_period_s > 0.0) {
    ripple = params_.voltage_ripple_v *
      std::sin(2.0 * units::PI * elapsed_s_ / params_.ripple_period_s);
  }

  output_.regulating = true;
  output_.bus_voltage_v = params_.voltage_setpoint_v + ripple;
  output_.available_power_w = array_power_w * std::clamp(params_.efficiency, 0.0, 1.0);
  output_.available_current_a =
    output_.available_power_w / std::max(output_.bus_voltage_v, 1.0e-3);
  return output_;
}

}  // namespace solar
}  // namespace ssos_eps
