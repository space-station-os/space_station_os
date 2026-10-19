#include "ssos_gnc/flight/actuators/cmg_steering.hpp"

#include <cmath>

namespace ssos_gnc
{

namespace flight
{

CmgSteering::CmgSteering() = default;

SteeringResult CmgSteering::solve(
  const Vector3 & torque_cmd,
  const Vector4 & gimbal,
  const Vector3 & body_rate) const
{
  SteeringResult out;

  const Vector3 h = kinematics_.momentum(gimbal);

  const Vector3 rhs = -(torque_cmd + body_rate.cross(h));
  Vector4 rate = kinematics_.pseudo_inverse(gimbal) * rhs;

  if (!rate.allFinite()) {
    rate.setZero();
    out.rate_limited = true;
  }

  for (int i = 0; i < common::kNumCmg; ++i) {
    if (rate(i) > rate_limit_) {
      rate(i) = rate_limit_;
      out.rate_limited = true;
    } else if (rate(i) < -rate_limit_) {
      rate(i) = -rate_limit_;
      out.rate_limited = true;
    }
  }

  out.gimbal_rate = rate;
  out.manipulability = kinematics_.manipulability(gimbal);
  out.min_singular_value = kinematics_.min_singular_value(gimbal);
  return out;
}  // namespace flight

double CmgSteering::momentum_fraction(const Vector4 & gimbal) const
{
  const double envelope = kinematics_.momentum_envelope();
  if (envelope <= 0.0) {return 0.0;}
  return kinematics_.momentum(gimbal).norm() / envelope;
}  // namespace ssos_gnc
}
}
