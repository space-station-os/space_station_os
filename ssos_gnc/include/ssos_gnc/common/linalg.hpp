#ifndef SSOS_GNC__COMMON__LINALG_HPP_
#define SSOS_GNC__COMMON__LINALG_HPP_

#include <algorithm>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/SVD>

namespace ssos_gnc
{

namespace common
{

template<typename MatrixType>
MatrixType pseudo_inverse(
  const MatrixType & a,
  double epsilon = std::numeric_limits<double>::epsilon())
{
  if (a.size() == 0) {return MatrixType(a.cols(), a.rows());}

  Eigen::JacobiSVD<MatrixType> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto & s = svd.singularValues();
  const double s_max = (s.size() > 0) ? s.maxCoeff() : 0.0;
  const double tol = epsilon * static_cast<double>(std::max(a.cols(), a.rows())) * s_max;

  Eigen::ArrayXd inv = s.array();
  for (int i = 0; i < inv.size(); ++i) {
    inv(i) = (std::abs(inv(i)) > tol) ? 1.0 / inv(i) : 0.0;
  }
  return svd.matrixV() * inv.matrix().asDiagonal() * svd.matrixU().adjoint();
}  // namespace common

template<typename MatrixType>
double smallest_singular_value(const MatrixType & a)
{
  if (a.size() == 0) {return 0.0;}
  Eigen::JacobiSVD<MatrixType> svd(a);
  const auto & s = svd.singularValues();
  return (s.size() > 0) ? s.minCoeff() : 0.0;
}  // namespace ssos_gnc

template<typename MatrixType>
double condition_number(const MatrixType & a)
{
  if (a.size() == 0) {return std::numeric_limits<double>::infinity();}
  Eigen::JacobiSVD<MatrixType> svd(a);
  const auto & s = svd.singularValues();
  if (s.size() == 0) {return std::numeric_limits<double>::infinity();}
  const double s_min = s.minCoeff();
  if (s_min <= 0.0) {return std::numeric_limits<double>::infinity();}
  return s.maxCoeff() / s_min;
}
}
}

#endif  // SSOS_GNC__COMMON__LINALG_HPP_
