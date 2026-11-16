#ifndef SSOS_GNC__PLANT__INERTIA_HPP_
#define SSOS_GNC__PLANT__INERTIA_HPP_

#include <Eigen/Dense>

namespace ssos_gnc
{

namespace plant
{

class Inertia
{
public:
  Inertia();
  explicit Inertia(const Eigen::Matrix3d & tensor);

  static Inertia from_principal(double ixx, double iyy, double izz);

  static Inertia from_elements(
    double ixx, double iyy, double izz,
    double ixy, double ixz, double iyz);

  bool is_valid() const {return valid_;}

  const Eigen::Matrix3d & tensor() const {return tensor_;}
  const Eigen::Matrix3d & inverse() const {return inverse_;}

  Eigen::Vector3d momentum(const Eigen::Vector3d & omega) const {return tensor_ * omega;}

private:
  void recompute();

  Eigen::Matrix3d tensor_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d inverse_{Eigen::Matrix3d::Identity()};
  bool valid_{true};
};

Inertia default_inertia();
}  // namespace plant
}  // namespace ssos_gnc

#endif  // SSOS_GNC__PLANT__INERTIA_HPP_
