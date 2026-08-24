#include "ssos_gnc/flight/estimation/attitude_estimator.hpp"

namespace ssos_gnc
{

namespace flight
{

AttitudeEstimator::AttitudeEstimator() = default;

void AttitudeEstimator::reset()
{
  state_ = EstimatorOutput{};
  initialized_ = false;
}  // namespace flight

EstimatorOutput AttitudeEstimator::update(double dt, const EstimatorInput & input)
{
  state_.body_rate = input.body_rate;
  state_.coasting = !input.attitude_valid;

  if (!filtering_enabled_) {
    if (input.attitude_valid) {
      state_.attitude = common::quat_normalized(input.attitude);
    }
    return state_;
  }

  if (!initialized_) {
    if (input.attitude_valid) {
      state_.attitude = common::quat_normalized(input.attitude);
      initialized_ = true;
    }
    return state_;
  }

  Quaternion propagated = common::quat_integrate(state_.attitude, input.body_rate, dt);

  if (input.attitude_valid) {
    const Quaternion residual = common::quat_error(propagated, input.attitude);
    const Vector3 correction = correction_gain_ * 2.0 * residual.vec();
    propagated = common::quat_normalized(
      propagated * common::quat_from_rotation_vector(correction));
  }

  state_.attitude = propagated;
  return state_;
}  // namespace ssos_gnc
}
}
