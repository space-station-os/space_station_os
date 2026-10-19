#include "ssos_gnc/plant/sensor_error_model.hpp"

#include <cmath>

namespace ssos_gnc
{

namespace plant
{

SensorErrorModel::SensorErrorModel(std::uint32_t seed)
: rng_(seed)
{
}  // namespace plant

void SensorErrorModel::reset(std::uint32_t seed)
{
  rng_.seed(seed);
  accumulated_bias_.setZero();
  last_rate_.setZero();
  rate_stuck_ = false;
}  // namespace ssos_gnc

double SensorErrorModel::gaussian(double stddev)
{
  if (stddev <= 0.0) {return 0.0;}
  return unit_normal_(rng_) * stddev;
}

Vector3 SensorErrorModel::measure_rate(const Vector3 & true_rate, double dt)
{
  if (rate_stuck_) {
    return last_rate_;
  }

  if (dt > 0.0 && imu_.bias_random_walk > 0.0) {
    const double sigma = imu_.bias_random_walk * std::sqrt(dt);
    accumulated_bias_ += Vector3(gaussian(sigma), gaussian(sigma), gaussian(sigma));
  }

  Vector3 measured = true_rate * (1.0 + imu_.scale_factor_error);
  measured += imu_.bias + accumulated_bias_;
  measured += Vector3(
    gaussian(imu_.noise_stddev),
    gaussian(imu_.noise_stddev),
    gaussian(imu_.noise_stddev));

  last_rate_ = measured;
  return measured;
}

bool SensorErrorModel::measure_attitude(const Quaternion & true_attitude, Quaternion & out)
{
  if (star_.dropout_probability > 0.0 && unit_uniform_(rng_) < star_.dropout_probability) {
    return false;
  }

  const double sigma_rad = star_.noise_arcsec * (common::kPi / (180.0 * 3600.0));
  const Vector3 err(gaussian(sigma_rad), gaussian(sigma_rad), gaussian(sigma_rad));

  out = common::quat_normalized(
    common::quat_normalized(true_attitude) * common::quat_from_rotation_vector(err));
  return true;
}
}
}
