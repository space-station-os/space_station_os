#include "ssos_gnc/flight/control/attitude_controller.hpp"

#include <algorithm>
#include <cmath>

namespace ssos_gnc
{

namespace flight
{

AttitudeController::AttitudeController()
: params_(default_gnc_parameters())
{
}  // namespace flight

AttitudeController::AttitudeController(const GncParameters & params)
: params_(params)
{
}  // namespace ssos_gnc

void AttitudeController::reset()
{
  angle_filter_state_.setZero();
  rate_filter_state_.setZero();
  dead_zone_active_ = false;
}

AttitudeError AttitudeController::compute_error(
  const Quaternion & estimated,
  const Quaternion & reference,
  const Vector3 & body_rate)
{
  AttitudeError e;

  Quaternion q_err = common::quat_canonical(
    common::quat_normalized(estimated).conjugate() * common::quat_normalized(reference));

  const double w = std::clamp(q_err.w(), -1.0, 1.0);
  const double sin_half = std::sqrt(std::max(0.0, 1.0 - w * w));
  const double theta = 2.0 * std::atan2(sin_half, w);

  Vector3 axis = Vector3::Zero();
  if (sin_half > 1e-12) {
    axis = q_err.vec() / sin_half;
  }

  e.vector = theta * axis;
  e.rate = body_rate;
  e.angle_rad = e.vector.norm();
  return e;
}

ControlTorques AttitudeController::compute_cmg(
  const AttitudeError & error,
  const Vector3 & cmg_momentum,
  bool unload_active) const
{
  ControlTorques out;

  Vector3 torque = params_.cmg.kp * error.vector + params_.cmg.kd * (-error.rate);

  if (!unload_active) {
    out.cmg = torque;
    return out;
  }

  const Vector3 t_bias = -params_.unload.gain * cmg_momentum;
  const double norm = t_bias.norm();

  if (norm <= 1e-12) {
    out.cmg = torque;
    return out;
  }

  const Vector3 bias_dir = t_bias / norm;

  const Matrix3 projector = Matrix3::Identity() - bias_dir * bias_dir.transpose();
  const Vector3 t_att = projector * torque;

  out.cmg = t_att;
  out.thruster = t_bias + (torque - t_att);
  out.unloaded_bias = t_bias;
  return out;
}

ControlTorques AttitudeController::compute_thruster(const AttitudeError & error)
{
  ControlTorques out;

  if (params_.filter.angle_alpha > 0.0) {
    angle_filter_state_ =
      params_.filter.angle_alpha * angle_filter_state_ +
      (1.0 - params_.filter.angle_alpha) * error.vector;
  } else {
    angle_filter_state_ = error.vector;
  }

  if (params_.filter.rate_alpha > 0.0) {
    rate_filter_state_ =
      params_.filter.rate_alpha * rate_filter_state_ +
      (1.0 - params_.filter.rate_alpha) * error.rate;
  } else {
    rate_filter_state_ = error.rate;
  }

  const double angle_deg = common::rad_to_deg(error.vector.norm());
  const double rate_dps = common::rad_to_deg(error.rate.norm());

  if (params_.dead_zone.enabled) {
    if (!dead_zone_active_ &&
      angle_deg <= params_.dead_zone.angle_off_deg &&
      rate_dps <= params_.dead_zone.rate_off_dps)
    {
      dead_zone_active_ = true;
    } else if (dead_zone_active_ &&  // NOLINT(readability/braces)
      (angle_deg > params_.dead_zone.angle_on_deg ||
      rate_dps > params_.dead_zone.rate_on_dps))
    {
      dead_zone_active_ = false;
    }
  } else {
    dead_zone_active_ = false;
  }

  out.dead_zone_active = dead_zone_active_;

  if (dead_zone_active_) {
    return out;
  }

  out.thruster =
    params_.thruster.kp * angle_filter_state_ +
    params_.thruster.kd * (-rate_filter_state_);
  return out;
}
}
}
