#ifndef SSOS_GNC__PLANT__DISTURBANCE_TORQUES_HPP_
#define SSOS_GNC__PLANT__DISTURBANCE_TORQUES_HPP_

#include "ssos_gnc/common/frames.hpp"
#include "ssos_gnc/common/quaternion.hpp"
#include "ssos_gnc/plant/inertia.hpp"

namespace ssos_gnc
{

namespace plant
{

using common::Quaternion;
using common::Vector3;

struct EnvironmentConditions
{
  double altitude_km{400.0};
  double solar_flux_w_m2{1361.0};
  bool in_eclipse{false};

  Quaternion attitude_lvlh{Quaternion::Identity()};
};

struct DisturbanceParams
{
  double drag_coefficient{2.2};
  double frontal_area_m2{1500.0};
  double center_of_pressure_offset_m{2.0};

  double srp_area_m2{2500.0};
  double srp_reflectivity{1.3};
  double srp_center_offset_m{1.5};

  bool enable_gravity_gradient{true};
  bool enable_aero_drag{true};
  bool enable_srp{true};
};

struct DisturbanceResult
{
  Vector3 gravity_gradient{Vector3::Zero()};
  Vector3 aero_drag{Vector3::Zero()};
  Vector3 solar_pressure{Vector3::Zero()};
  Vector3 total{Vector3::Zero()};
};

class DisturbanceTorques
{
public:
  DisturbanceTorques();
  explicit DisturbanceTorques(const DisturbanceParams & params);

  DisturbanceResult compute(const EnvironmentConditions & env, const Inertia & inertia) const;

  void set_parameters(const DisturbanceParams & p) {params_ = p;}
  const DisturbanceParams & parameters() const {return params_;}

  static double atmospheric_density(double altitude_km);

private:
  DisturbanceParams params_;
};
}  // namespace plant
}  // namespace ssos_gnc

#endif  // SSOS_GNC__PLANT__DISTURBANCE_TORQUES_HPP_
