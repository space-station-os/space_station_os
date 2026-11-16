#ifndef SSOS_GNC__PLANT__SENSOR_ERROR_MODEL_HPP_
#define SSOS_GNC__PLANT__SENSOR_ERROR_MODEL_HPP_

#include <cstdint>
#include <random>

#include "ssos_gnc/common/quaternion.hpp"

namespace ssos_gnc
{

namespace plant
{

using common::Quaternion;
using common::Vector3;

struct ImuErrorParams
{
  Vector3 bias{Vector3::Zero()};
  double noise_stddev{1.0e-5};
  double bias_random_walk{1.0e-9};
  double scale_factor_error{0.0};
};

struct StarTrackerErrorParams
{
  double noise_arcsec{10.0};
  double dropout_probability{0.0};
};

class SensorErrorModel
{
public:
  explicit SensorErrorModel(std::uint32_t seed = 12345u);

  Vector3 measure_rate(const Vector3 & true_rate, double dt);

  bool measure_attitude(const Quaternion & true_attitude, Quaternion & out);

  void set_imu_params(const ImuErrorParams & p) {imu_ = p;}
  void set_star_tracker_params(const StarTrackerErrorParams & p) {star_ = p;}
  const ImuErrorParams & imu_params() const {return imu_;}

  const Vector3 & current_bias() const {return accumulated_bias_;}

  void reset(std::uint32_t seed = 12345u);

  void set_rate_stuck(bool stuck) {rate_stuck_ = stuck;}

private:
  double gaussian(double stddev);

  ImuErrorParams imu_;
  StarTrackerErrorParams star_;
  std::mt19937 rng_;
  std::normal_distribution<double> unit_normal_{0.0, 1.0};
  std::uniform_real_distribution<double> unit_uniform_{0.0, 1.0};
  Vector3 accumulated_bias_{Vector3::Zero()};
  Vector3 last_rate_{Vector3::Zero()};
  bool rate_stuck_{false};
};
}  // namespace plant
}  // namespace ssos_gnc

#endif  // SSOS_GNC__PLANT__SENSOR_ERROR_MODEL_HPP_
