#include "ssos_gnc/common/frames.hpp"

namespace ssos_gnc
{

namespace common
{

Matrix3 eci_to_lvlh_matrix(const OrbitState & orbit)
{
  const Vector3 & r = orbit.position_eci;
  const Vector3 & v = orbit.velocity_eci;

  const double r_norm = r.norm();
  if (r_norm < 1.0) {return Matrix3::Identity();}

  const Vector3 z_lvlh = -r / r_norm;

  const Vector3 h = r.cross(v);
  const double h_norm = h.norm();
  if (h_norm < 1e-6) {return Matrix3::Identity();}
  const Vector3 y_lvlh = -h / h_norm;

  const Vector3 x_lvlh = y_lvlh.cross(z_lvlh).normalized();

  Matrix3 m;
  m.row(0) = x_lvlh.transpose();
  m.row(1) = y_lvlh.transpose();
  m.row(2) = z_lvlh.transpose();
  return m;
}  // namespace common

Quaternion eci_to_lvlh(const OrbitState & orbit)
{
  return quat_normalized(Quaternion(eci_to_lvlh_matrix(orbit)));
}  // namespace ssos_gnc

Matrix3 eci_to_ecef_matrix(double gmst_rad)
{
  const double c = std::cos(gmst_rad);
  const double s = std::sin(gmst_rad);
  Matrix3 m;
  m << c, s, 0.0,
    -s, c, 0.0,
    0.0, 0.0, 1.0;
  return m;
}

Vector3 lvlh_rate(const OrbitState & orbit)
{
  const Vector3 & r = orbit.position_eci;
  const Vector3 & v = orbit.velocity_eci;
  const double r2 = r.squaredNorm();
  if (r2 < 1.0) {return Vector3::Zero();}

  const Vector3 omega_eci = r.cross(v) / r2;
  return eci_to_lvlh_matrix(orbit) * omega_eci;
}

OrbitState circular_orbit_from_altitude(double altitude_km, double phase_rad)
{
  OrbitState s;
  const double radius = kEarthRadius + km_to_m(altitude_km);
  if (radius < 1.0) {return s;}

  const double speed = std::sqrt(kEarthMu / radius);
  s.position_eci = Vector3(radius * std::cos(phase_rad), radius * std::sin(phase_rad), 0.0);
  s.velocity_eci = Vector3(-speed * std::sin(phase_rad), speed * std::cos(phase_rad), 0.0);
  return s;
}
}
}
