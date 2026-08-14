#include "ssos_eclss/ars/blower_model.hpp"

#include <algorithm>
#include <cmath>

namespace ssos_eclss
{
namespace ars
{

BlowerModel::BlowerModel(const BlowerParams & params) : params_(params) {}

double BlowerModel::fan_head(double rpm, double flow_m3s) const
{
  const double speed_ratio = rpm / params_.rated_rpm;
  const double head = params_.shutoff_dp * speed_ratio * speed_ratio -
                      params_.curve_quad * flow_m3s * flow_m3s;
  return std::max(head, 0.0);
}

BlowerResult BlowerModel::solve(double system_resistance, BlowerControl control,
                                double setpoint) const
{
  BlowerResult r{};

  if (control == BlowerControl::CONSTANT_FLOW) {
    r.flow_m3s = std::max(setpoint, 0.0);
    r.delta_p = system_resistance * r.flow_m3s * r.flow_m3s;
    // Invert fan curve for the rpm that delivers this flow at this head.
    const double needed = (r.delta_p + params_.curve_quad * r.flow_m3s * r.flow_m3s) /
                          params_.shutoff_dp;
    r.rpm = params_.rated_rpm * std::sqrt(std::max(needed, 0.0));
    r.rpm = std::min(r.rpm, params_.max_rpm);
  } else {
    // CONSTANT_RPM: intersect fan curve with system curve.
    // shutoff*s^2 - a2*Q^2 = R*Q^2  ->  Q^2 = shutoff*s^2 / (a2 + R)
    const double rpm = std::min(setpoint, params_.max_rpm);
    const double s = rpm / params_.rated_rpm;
    const double q2 = params_.shutoff_dp * s * s /
                      (params_.curve_quad + system_resistance);
    r.flow_m3s = std::sqrt(std::max(q2, 0.0));
    r.delta_p = system_resistance * q2;
    r.rpm = rpm;
  }

  // Shaft power estimate: P = dP * Q / efficiency (assume 0.55 blower efficiency).
  const double eff = 0.55;
  r.power_w = r.delta_p * r.flow_m3s / eff;
  return r;
}

}  // namespace ars
}  // namespace ssos_eclss
