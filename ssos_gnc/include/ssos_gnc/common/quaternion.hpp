#ifndef SSOS_GNC__COMMON__QUATERNION_HPP_
#define SSOS_GNC__COMMON__QUATERNION_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ssos_gnc/common/units.hpp"

namespace ssos_gnc
{

namespace common
{

using Quaternion = Eigen::Quaterniond;
using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;

inline Quaternion quat_identity() {return Quaternion::Identity();}

inline Quaternion quat_normalized(const Quaternion & q)
{
  const double n = q.norm();
  if (n < 1e-12) {return Quaternion::Identity();}
  Quaternion out = q;
  out.coeffs() /= n;
  return out;
}  // namespace common

inline Quaternion quat_canonical(const Quaternion & q)
{
  if (q.w() < 0.0) {
    Quaternion out;
    out.coeffs() = -q.coeffs();
    return out;
  }
  return q;
}  // namespace ssos_gnc

inline Quaternion quat_error(const Quaternion & actual, const Quaternion & reference)
{
  return quat_canonical(quat_normalized(actual).conjugate() * quat_normalized(reference));
}

inline Vector3 quat_error_vector(const Quaternion & actual, const Quaternion & reference)
{
  return 2.0 * quat_error(actual, reference).vec();
}

inline double quat_angle(const Quaternion & q)
{
  const Quaternion c = quat_canonical(quat_normalized(q));
  return 2.0 * std::atan2(c.vec().norm(), c.w());
}

inline Quaternion quat_derivative(const Quaternion & q, const Vector3 & omega_body)
{
  const Quaternion omega_quat(0.0, omega_body.x(), omega_body.y(), omega_body.z());
  Quaternion dq;
  dq.coeffs() = 0.5 * (q * omega_quat).coeffs();
  return dq;
}

inline Quaternion quat_integrate(const Quaternion & q, const Vector3 & omega_body, double dt)
{
  Quaternion out = q;
  out.coeffs() += quat_derivative(q, omega_body).coeffs() * dt;
  return quat_normalized(out);
}

inline Matrix3 skew(const Vector3 & v)
{
  Matrix3 m;
  m << 0.0, -v.z(), v.y(),
    v.z(), 0.0, -v.x(),
    -v.y(), v.x(), 0.0;
  return m;
}

inline Quaternion quat_from_rotation_vector(const Vector3 & rv)
{
  const double angle = rv.norm();
  if (angle < 1e-12) {return Quaternion::Identity();}
  return Quaternion(Eigen::AngleAxisd(angle, rv / angle));
}
}
}

#endif  // SSOS_GNC__COMMON__QUATERNION_HPP_
