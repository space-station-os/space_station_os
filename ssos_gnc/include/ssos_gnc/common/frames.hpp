#ifndef SSOS_GNC__COMMON__FRAMES_HPP_
#define SSOS_GNC__COMMON__FRAMES_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ssos_gnc/common/quaternion.hpp"
#include "ssos_gnc/common/units.hpp"

namespace ssos_gnc
{

namespace common
{

struct OrbitState
{
  Vector3 position_eci{Vector3::Zero()};
  Vector3 velocity_eci{Vector3::Zero()};
};

Matrix3 eci_to_lvlh_matrix(const OrbitState & orbit);

Quaternion eci_to_lvlh(const OrbitState & orbit);

Matrix3 eci_to_ecef_matrix(double gmst_rad);

inline double gmst_at(double gmst0_rad, double seconds)
{
  return wrap_two_pi(gmst0_rad + kEarthRotRate * seconds);
}  // namespace common

Vector3 lvlh_rate(const OrbitState & orbit);

inline double mean_motion(double radius_m)
{
  if (radius_m < 1.0) {return 0.0;}
  return std::sqrt(kEarthMu / (radius_m * radius_m * radius_m));
}  // namespace ssos_gnc

OrbitState circular_orbit_from_altitude(double altitude_km, double phase_rad = 0.0);
}
}

#endif  // SSOS_GNC__COMMON__FRAMES_HPP_
