#include "ssos_eclss/ars/valve_model.hpp"

#include <algorithm>
#include <cmath>

namespace ssos_eclss
{
namespace ars
{

ValveModel::ValveModel(const ValveParams & params) : params_(params) {}

void ValveModel::update(double dt)
{
  if (params_.stroke_time_s <= 0.0) {
    position_ = target_;
    return;
  }
  const double max_step = dt / params_.stroke_time_s;  // fraction per dt
  const double err = target_ - position_;
  if (std::abs(err) <= max_step) {
    position_ = target_;
  } else {
    position_ += std::copysign(max_step, err);
  }
  position_ = clamp01(position_);
}

double ValveModel::flow(double delta_p_pa) const
{
  const double sign = (delta_p_pa >= 0.0) ? 1.0 : -1.0;
  return position_ * params_.cv * sign * std::sqrt(std::abs(delta_p_pa));
}

double ValveModel::repressurize(double dt, double current_pressure,
                                double target_pressure) const
{
  if (params_.repress_time_s <= 0.0) {
    return target_pressure;
  }
  // First-order approach: matches the ~15 s 0->800 torr fill of the EDU.
  const double alpha = 1.0 - std::exp(-dt / params_.repress_time_s);
  return current_pressure + alpha * (target_pressure - current_pressure);
}

}  // namespace ars
}  // namespace ssos_eclss
