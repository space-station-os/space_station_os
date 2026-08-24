#ifndef SSOS_GNC__PLANT__RIGID_BODY_HPP_
#define SSOS_GNC__PLANT__RIGID_BODY_HPP_

#include "ssos_gnc/common/cmg_kinematics.hpp"
#include "ssos_gnc/common/numerical/rk4_integrator.hpp"
#include "ssos_gnc/common/quaternion.hpp"
#include "ssos_gnc/plant/inertia.hpp"

namespace ssos_gnc
{

namespace plant
{

using common::Quaternion;
using common::Vector3;
using common::Vector4;

struct RigidBodyState
{
  Quaternion attitude{Quaternion::Identity()};
  Vector3 omega{Vector3::Zero()};
  Vector4 gimbal{Vector4::Zero()};

  RigidBodyState operator+(const RigidBodyState & rhs) const;
  RigidBodyState operator*(double s) const;

  void normalize();
};

struct RigidBodyInput
{
  Vector3 external_torque{Vector3::Zero()};
  Vector4 gimbal_rate_cmd{Vector4::Zero()};

  Vector3 cmg_torque_cmd{Vector3::Zero()};
};

class RigidBody
{
public:
  RigidBody();
  explicit RigidBody(const Inertia & inertia);

  void step(double dt, const RigidBodyInput & input);

  void reset();
  void set_state(const RigidBodyState & s);
  void set_inertia(const Inertia & i) {inertia_ = i;}
  void set_max_substep(double s) {max_substep_s_ = (s > 0.0) ? s : max_substep_s_;}

  void set_gimbal_rate_limit(double rad_s) {gimbal_rate_limit_ = rad_s;}

  void set_momentum_conserving(bool enabled) {momentum_conserving_ = enabled;}
  bool momentum_conserving() const {return momentum_conserving_;}

  const RigidBodyState & state() const {return state_;}
  const Inertia & inertia() const {return inertia_;}

  Vector3 cmg_momentum() const;

  Vector3 total_momentum() const;

  bool gimbal_rate_saturated() const {return gimbal_saturated_;}

private:
  RigidBodyState derivative(const RigidBodyState & s, const RigidBodyInput & in) const;

  Inertia inertia_;
  common::CmgKinematics cmg_;
  RigidBodyState state_;
  double max_substep_s_{0.02};
  double gimbal_rate_limit_{0.5};
  bool gimbal_saturated_{false};
  bool momentum_conserving_{false};
};
}  // namespace plant
}  // namespace ssos_gnc

#endif  // SSOS_GNC__PLANT__RIGID_BODY_HPP_
