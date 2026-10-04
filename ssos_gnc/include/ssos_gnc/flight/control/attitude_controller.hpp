#ifndef SSOS_GNC__FLIGHT__CONTROL__ATTITUDE_CONTROLLER_HPP_
#define SSOS_GNC__FLIGHT__CONTROL__ATTITUDE_CONTROLLER_HPP_

#include "ssos_gnc/common/quaternion.hpp"
#include "ssos_gnc/flight/control/control_parameters.hpp"

namespace ssos_gnc
{

namespace flight
{

using common::Quaternion;
using common::Vector3;
using common::Matrix3;

struct AttitudeError
{
  Vector3 vector{Vector3::Zero()};
  Vector3 rate{Vector3::Zero()};
  double angle_rad{0.0};
};

struct ControlTorques
{
  Vector3 cmg{Vector3::Zero()};
  Vector3 thruster{Vector3::Zero()};
  Vector3 unloaded_bias{Vector3::Zero()};
  bool dead_zone_active{false};
};

class AttitudeController
{
public:
  AttitudeController();
  explicit AttitudeController(const GncParameters & params);

  static AttitudeError compute_error(
    const Quaternion & estimated,
    const Quaternion & reference,
    const Vector3 & body_rate);

  ControlTorques compute_cmg(
    const AttitudeError & error,
    const Vector3 & cmg_momentum,
    bool unload_active) const;

  ControlTorques compute_thruster(const AttitudeError & error);

  void set_parameters(const GncParameters & p) {params_ = p;}
  const GncParameters & parameters() const {return params_;}

  void reset();

  bool dead_zone_active() const {return dead_zone_active_;}

private:
  GncParameters params_;

  Vector3 angle_filter_state_{Vector3::Zero()};
  Vector3 rate_filter_state_{Vector3::Zero()};

  bool dead_zone_active_{false};
};
}  // namespace flight
}  // namespace ssos_gnc

#endif  // SSOS_GNC__FLIGHT__CONTROL__ATTITUDE_CONTROLLER_HPP_
