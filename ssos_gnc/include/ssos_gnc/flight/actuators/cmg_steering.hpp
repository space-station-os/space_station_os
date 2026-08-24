#ifndef SSOS_GNC__FLIGHT__ACTUATORS__CMG_STEERING_HPP_
#define SSOS_GNC__FLIGHT__ACTUATORS__CMG_STEERING_HPP_

#include "ssos_gnc/common/cmg_kinematics.hpp"
#include "ssos_gnc/common/quaternion.hpp"

namespace ssos_gnc
{

namespace flight
{

using common::Vector3;
using common::Vector4;

struct SteeringResult
{
  Vector4 gimbal_rate{Vector4::Zero()};
  double manipulability{0.0};
  double min_singular_value{0.0};
  bool rate_limited{false};
};

class CmgSteering
{
public:
  CmgSteering();

  SteeringResult solve(
    const Vector3 & torque_cmd,
    const Vector4 & gimbal,
    const Vector3 & body_rate) const;

  Vector3 momentum(const Vector4 & gimbal) const {return kinematics_.momentum(gimbal);}

  double momentum_fraction(const Vector4 & gimbal) const;

  void set_rate_limit(double rad_s) {rate_limit_ = rad_s;}
  const common::CmgKinematics & kinematics() const {return kinematics_;}

private:
  common::CmgKinematics kinematics_;
  double rate_limit_{0.5};
};
}  // namespace flight
}  // namespace ssos_gnc

#endif  // SSOS_GNC__FLIGHT__ACTUATORS__CMG_STEERING_HPP_
