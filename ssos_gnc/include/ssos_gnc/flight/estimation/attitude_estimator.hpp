#ifndef SSOS_GNC__FLIGHT__ESTIMATION__ATTITUDE_ESTIMATOR_HPP_
#define SSOS_GNC__FLIGHT__ESTIMATION__ATTITUDE_ESTIMATOR_HPP_

#include "ssos_gnc/common/quaternion.hpp"

namespace ssos_gnc
{

namespace flight
{

using common::Quaternion;
using common::Vector3;

struct EstimatorInput
{
  Vector3 body_rate{Vector3::Zero()};
  Quaternion attitude{Quaternion::Identity()};
  bool attitude_valid{true};
};

struct EstimatorOutput
{
  Quaternion attitude{Quaternion::Identity()};
  Vector3 body_rate{Vector3::Zero()};
  bool coasting{false};
};

class AttitudeEstimator
{
public:
  AttitudeEstimator();

  void set_filtering_enabled(bool enabled) {filtering_enabled_ = enabled;}
  bool filtering_enabled() const {return filtering_enabled_;}

  void set_correction_gain(double gain) {correction_gain_ = common::clamp(gain, 0.0, 1.0);}

  EstimatorOutput update(double dt, const EstimatorInput & input);

  void reset();
  const EstimatorOutput & state() const {return state_;}

private:
  EstimatorOutput state_;
  bool filtering_enabled_{false};
  double correction_gain_{0.02};
  bool initialized_{false};
};
}  // namespace flight
}  // namespace ssos_gnc

#endif  // SSOS_GNC__FLIGHT__ESTIMATION__ATTITUDE_ESTIMATOR_HPP_
