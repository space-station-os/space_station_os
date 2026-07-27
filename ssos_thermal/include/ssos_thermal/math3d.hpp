#ifndef SSOS_THERMAL__MATH3D_HPP_
#define SSOS_THERMAL__MATH3D_HPP_

#include <cmath>

// Minimal std-only replacement for the btVector3 / btQuaternion subset the
// legacy space_station_thermal_control sun_vector/solar_heat_node use. Drops
// the Bullet Physics dependency for what was pure vector/quaternion math,
// not collision physics.

namespace ssos_thermal
{
namespace math3d
{

struct Vector3
{
  double x, y, z;

  explicit Vector3(double x_ = 0.0, double y_ = 0.0, double z_ = 0.0)
  : x(x_), y(y_), z(z_) {}

  Vector3 operator-(const Vector3 & o) const
  {
    return Vector3(x - o.x, y - o.y, z - o.z);
  }

  double dot(const Vector3 & o) const {return x * o.x + y * o.y + z * o.z;}

  double length() const {return std::sqrt(dot(*this));}

  Vector3 normalized() const
  {
    const double len = length();
    return len > 0.0 ? Vector3(x / len, y / len, z / len) : Vector3();
  }

  void normalize() {*this = normalized();}
};

struct Quaternion
{
  double x, y, z, w;

  explicit Quaternion(double x_ = 0.0, double y_ = 0.0, double z_ = 0.0, double w_ = 1.0)
  : x(x_), y(y_), z(z_), w(w_) {}

  // Conjugate -- equals the true inverse for the unit quaternions this is used with.
  Quaternion inverse() const {return Quaternion(-x, -y, -z, w);}

  Quaternion operator*(const Quaternion & o) const
  {
    return Quaternion(
      w * o.x + x * o.w + y * o.z - z * o.y,
      w * o.y - x * o.z + y * o.w + z * o.x,
      w * o.z + x * o.y - y * o.x + z * o.w,
      w * o.w - x * o.x - y * o.y - z * o.z);
  }
};

}  // namespace math3d
}  // namespace ssos_thermal

#endif  // SSOS_THERMAL__MATH3D_HPP_
