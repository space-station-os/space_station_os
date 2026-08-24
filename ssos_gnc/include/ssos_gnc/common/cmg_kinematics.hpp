#ifndef SSOS_GNC__COMMON__CMG_KINEMATICS_HPP_
#define SSOS_GNC__COMMON__CMG_KINEMATICS_HPP_

#include <vector>

#include <Eigen/Dense>

#include "ssos_gnc/common/linalg.hpp"

namespace ssos_gnc
{

namespace common
{

constexpr int kNumCmg = 4;

using Vector4 = Eigen::Matrix<double, 4, 1>;
using Matrix3x4 = Eigen::Matrix<double, 3, 4>;
using Matrix4x3 = Eigen::Matrix<double, 4, 3>;

class CmgKinematics
{
public:
  CmgKinematics();

  Eigen::Vector3d momentum(const Vector4 & delta) const;

  Matrix3x4 jacobian(const Vector4 & delta) const;

  Matrix4x3 pseudo_inverse(const Vector4 & delta) const;

  Matrix4x3 damped_pseudo_inverse(const Vector4 & delta) const;

  double manipulability(const Vector4 & delta) const;

  double min_singular_value(const Vector4 & delta) const;

  double momentum_envelope() const;

private:
  mutable std::vector<double> w_;

  double wheel_momentum_{0.0};
};
}  // namespace common
}  // namespace ssos_gnc

#endif  // SSOS_GNC__COMMON__CMG_KINEMATICS_HPP_
