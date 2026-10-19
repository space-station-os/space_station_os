#include "ssos_eps/common/interpolation.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "ssos_eps/common/units.hpp"

namespace ssos_eps
{
namespace common
{

double interpolate(const std::vector<double> & xs, const std::vector<double> & ys,
                   double x)
{
  const std::size_t n = std::min(xs.size(), ys.size());
  if (n == 0) {
    return 0.0;
  }
  if (n == 1 || x <= xs.front()) {
    return ys.front();
  }
  if (x >= xs[n - 1]) {
    return ys[n - 1];
  }
  const auto it = std::upper_bound(xs.begin(), xs.begin() + n, x);
  const std::size_t hi = static_cast<std::size_t>(std::distance(xs.begin(), it));
  const std::size_t lo = hi - 1;
  const double span = xs[hi] - xs[lo];
  if (span <= 0.0) {
    return ys[lo];
  }
  const double w = (x - xs[lo]) / span;
  return ys[lo] + w * (ys[hi] - ys[lo]);
}

double cosine_rise(double x)
{
  if (x <= 0.0) {
    return 0.0;
  }
  if (x >= 1.0) {
    return 1.0;
  }
  return 0.5 * (1.0 - std::cos(units::PI * x));
}

}  // namespace common
}  // namespace ssos_eps
