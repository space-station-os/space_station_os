#include "ssos_gnc/plant/rigid_body.hpp"

#include <algorithm>

namespace ssos_gnc
{

namespace plant
{

RigidBodyState RigidBodyState::operator+(const RigidBodyState & rhs) const
{
  RigidBodyState out;
  out.attitude.coeffs() = attitude.coeffs() + rhs.attitude.coeffs();
  out.omega = omega + rhs.omega;
  out.gimbal = gimbal + rhs.gimbal;
  return out;
}  // namespace plant

RigidBodyState RigidBodyState::operator*(double s) const
{
  RigidBodyState out;
  out.attitude.coeffs() = attitude.coeffs() * s;
  out.omega = omega * s;
  out.gimbal = gimbal * s;
  return out;
}  // namespace ssos_gnc

void RigidBodyState::normalize()
{
  attitude = common::quat_normalized(attitude);
}

RigidBody::RigidBody()
: inertia_(default_inertia())
{
}

RigidBody::RigidBody(const Inertia & inertia)
: inertia_(inertia)
{
}

void RigidBody::reset()
{
  state_ = RigidBodyState{};
  gimbal_saturated_ = false;
}

void RigidBody::set_state(const RigidBodyState & s)
{
  state_ = s;
  state_.normalize();
}

Vector3 RigidBody::cmg_momentum() const
{
  return cmg_.momentum(state_.gimbal);
}

Vector3 RigidBody::total_momentum() const
{
  return inertia_.momentum(state_.omega) + cmg_momentum();
}

RigidBodyState RigidBody::derivative(const RigidBodyState & s, const RigidBodyInput & in) const
{
  RigidBodyState d;

  d.gimbal = in.gimbal_rate_cmd;

  if (momentum_conserving_) {
    const Vector3 h = cmg_.momentum(s.gimbal);
    const Vector3 h_dot = cmg_.jacobian(s.gimbal) * in.gimbal_rate_cmd;
    const Vector3 total_h = inertia_.momentum(s.omega) + h;
    const Vector3 rhs = in.external_torque - s.omega.cross(total_h) - h_dot;
    d.omega = inertia_.inverse() * rhs;
  } else {
    const Vector3 tau = in.external_torque + in.cmg_torque_cmd;
    const Vector3 rhs = tau - s.omega.cross(inertia_.momentum(s.omega));
    d.omega = inertia_.inverse() * rhs;
  }

  d.attitude = common::quat_derivative(s.attitude, s.omega);

  return d;
}

void RigidBody::step(double dt, const RigidBodyInput & input)
{
  if (dt <= 0.0) {return;}

  RigidBodyInput clamped = input;
  gimbal_saturated_ = false;
  for (int i = 0; i < common::kNumCmg; ++i) {
    const double r = clamped.gimbal_rate_cmd(i);
    if (r > gimbal_rate_limit_) {
      clamped.gimbal_rate_cmd(i) = gimbal_rate_limit_;
      gimbal_saturated_ = true;
    } else if (r < -gimbal_rate_limit_) {
      clamped.gimbal_rate_cmd(i) = -gimbal_rate_limit_;
      gimbal_saturated_ = true;
    }
  }

  const int n = common::numerical::substeps_for(dt, max_substep_s_);

  std::function<RigidBodyState(const RigidBodyState &, double)> f =
    [this, &clamped](const RigidBodyState & s, double) {
      return this->derivative(s, clamped);
    };

  state_ = common::numerical::rk4_integrate<RigidBodyState>(state_, 0.0, dt, n, f);
  state_.normalize();
}
}
}
