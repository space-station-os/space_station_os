#include "ssos_gnc/plant/inertia.hpp"

#include <Eigen/Eigenvalues>

namespace ssos_gnc
{

namespace plant
{

Inertia::Inertia()
{
  recompute();
}  // namespace plant

Inertia::Inertia(const Eigen::Matrix3d & tensor)
: tensor_(tensor)
{
  recompute();
}  // namespace ssos_gnc

Inertia Inertia::from_principal(double ixx, double iyy, double izz)
{
  Eigen::Matrix3d t = Eigen::Matrix3d::Zero();
  t(0, 0) = ixx;
  t(1, 1) = iyy;
  t(2, 2) = izz;
  return Inertia(t);
}

Inertia Inertia::from_elements(
  double ixx, double iyy, double izz,
  double ixy, double ixz, double iyz)
{
  Eigen::Matrix3d t;
  t << ixx, ixy, ixz,
    ixy, iyy, iyz,
    ixz, iyz, izz;
  return Inertia(t);
}

void Inertia::recompute()
{
  const double scale = std::max(1.0, tensor_.cwiseAbs().maxCoeff());
  const bool symmetric = ((tensor_ - tensor_.transpose()).cwiseAbs().maxCoeff() / scale) < 1e-9;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(tensor_);
  const bool positive_definite =
    (solver.info() == Eigen::Success) && (solver.eigenvalues().minCoeff() > 0.0);

  valid_ = symmetric && positive_definite;

  if (valid_) {
    inverse_ = tensor_.inverse();
  } else {
    inverse_ = Eigen::Matrix3d::Identity();
  }
}

Inertia default_inertia()
{
  return Inertia::from_principal(1.0e8, 8.0e7, 6.0e7);
}
}
}
