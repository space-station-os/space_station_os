#ifndef SSOS_EPS__COMMON__INTERPOLATION_HPP_
#define SSOS_EPS__COMMON__INTERPOLATION_HPP_

#include <vector>

// Table lookup and smooth transition helpers.

namespace ssos_eps
{
namespace common
{

/// Piecewise-linear interpolation of y(x). xs must be strictly increasing and
/// the same length as ys. Values outside the table are clamped to the ends.
double interpolate(const std::vector<double> & xs, const std::vector<double> & ys,
                   double x);

/// Half-cosine rise from 0 at x<=0 to 1 at x>=1 (zero slope at both ends).
double cosine_rise(double x);

}  // namespace common
}  // namespace ssos_eps

#endif  // SSOS_EPS__COMMON__INTERPOLATION_HPP_
