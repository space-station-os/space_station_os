#include "ssos_gnc/common/cmg_kinematics.hpp"

#include <algorithm>
#include <cmath>

extern "C" {
int hFunc(const double ** arg, double ** res, long long int * iw, double * w, int mem);
int jacobFunc(const double ** arg, double ** res, long long int * iw, double * w, int mem);
int pseudoInvFunc(const double ** arg, double ** res, long long int * iw, double * w, int mem);
int hFunc_work(long long int * sz_arg, long long int * sz_res, long long int * sz_iw, long long int * sz_w);
int jacobFunc_work(long long int * sz_arg, long long int * sz_res, long long int * sz_iw, long long int * sz_w);
int pseudoInvFunc_work(long long int * sz_arg, long long int * sz_res, long long int * sz_iw, long long int * sz_w);
}

namespace ssos_gnc
{

namespace common
{

namespace
{
long long int max_work_size()
{
  long long int sz_arg = 0, sz_res = 0, sz_iw = 0, sz_w = 0;
  long long int worst = 0;

  hFunc_work(&sz_arg, &sz_res, &sz_iw, &sz_w);
  worst = std::max(worst, sz_w);
  jacobFunc_work(&sz_arg, &sz_res, &sz_iw, &sz_w);
  worst = std::max(worst, sz_w);
  pseudoInvFunc_work(&sz_arg, &sz_res, &sz_iw, &sz_w);
  worst = std::max(worst, sz_w);

  return worst;
}  // namespace common
}  // namespace ssos_gnc

CmgKinematics::CmgKinematics()
{
  w_.assign(static_cast<std::size_t>(max_work_size()) + 1u, 0.0);

  const Matrix3x4 a = jacobian(Vector4::Zero());
  wheel_momentum_ = a.col(0).norm();
}

Eigen::Vector3d CmgKinematics::momentum(const Vector4 & delta) const
{
  const double * arg[kNumCmg] = {&delta(0), &delta(1), &delta(2), &delta(3)};

  double out[3] = {0.0, 0.0, 0.0};
  double * res[1] = {out};

  hFunc(arg, res, nullptr, w_.data(), 0);
  return Eigen::Vector3d(out[0], out[1], out[2]);
}

Matrix3x4 CmgKinematics::jacobian(const Vector4 & delta) const
{
  const double * arg[kNumCmg] = {&delta(0), &delta(1), &delta(2), &delta(3)};

  double out[12] = {0.0};
  double * res[1] = {out};

  jacobFunc(arg, res, nullptr, w_.data(), 0);
  return Eigen::Map<const Matrix3x4>(out);
}

Matrix4x3 CmgKinematics::pseudo_inverse(const Vector4 & delta) const
{
  const double * arg[kNumCmg] = {&delta(0), &delta(1), &delta(2), &delta(3)};

  double out[12] = {0.0};
  double * res[1] = {out};

  pseudoInvFunc(arg, res, nullptr, w_.data(), 0);
  return Eigen::Map<const Matrix4x3>(out);
}

Matrix4x3 CmgKinematics::damped_pseudo_inverse(const Vector4 & delta) const
{
  const Matrix3x4 a = jacobian(delta);
  const Eigen::MatrixXd a_dyn = a;
  const Eigen::MatrixXd pinv = ::ssos_gnc::common::pseudo_inverse<Eigen::MatrixXd>(a_dyn);
  return pinv;
}

double CmgKinematics::manipulability(const Vector4 & delta) const
{
  const Matrix3x4 a = jacobian(delta);
  const Eigen::Matrix3d aat = a * a.transpose();
  const double det = aat.determinant();
  return (det > 0.0) ? std::sqrt(det) : 0.0;
}

double CmgKinematics::min_singular_value(const Vector4 & delta) const
{
  const Eigen::MatrixXd a = jacobian(delta);
  return ::ssos_gnc::common::smallest_singular_value<Eigen::MatrixXd>(a);
}

double CmgKinematics::momentum_envelope() const
{
  return static_cast<double>(kNumCmg) * wheel_momentum_;
}
}
}
