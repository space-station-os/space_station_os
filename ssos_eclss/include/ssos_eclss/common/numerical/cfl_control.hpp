#ifndef SSOS_ECLSS__COMMON__NUMERICAL__CFL_CONTROL_HPP_
#define SSOS_ECLSS__COMMON__NUMERICAL__CFL_CONTROL_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

// Adaptive timestep control based on the CFL (advection) and diffusion-number
// stability limits. Header-only, no dependencies.

namespace ssos_eclss
{
namespace numerical
{

/// Maximum stable advection timestep for a uniform grid: dt <= cfl * dz / |v|.
/// @param velocity advection velocity [m/s]
/// @param dz       cell size [m]
/// @param cfl_max  target Courant number (default 0.5)
inline double advection_dt_limit(double velocity, double dz, double cfl_max = 0.5)
{
  const double v = std::abs(velocity);
  if (v < 1.0e-12) {
    return std::numeric_limits<double>::max();
  }
  return cfl_max * dz / v;
}

/// Maximum stable explicit diffusion timestep: dt <= d_max * dz^2 / D.
/// @param diffusivity D [m^2/s]
/// @param dz          cell size [m]
/// @param d_max       target diffusion number (default 0.5)
inline double diffusion_dt_limit(double diffusivity, double dz, double d_max = 0.5)
{
  if (diffusivity < 1.0e-12) {
    return std::numeric_limits<double>::max();
  }
  return d_max * dz * dz / diffusivity;
}

/// Number of equal substeps required to advance @p total_dt without exceeding
/// the stability-limited step @p dt_limit. Always returns at least 1.
inline std::size_t required_substeps(double total_dt, double dt_limit)
{
  if (dt_limit <= 0.0 || !std::isfinite(dt_limit)) {
    return 1;
  }
  const double n = std::ceil(total_dt / dt_limit);
  if (!std::isfinite(n) || n < 1.0) {
    return 1;
  }
  return static_cast<std::size_t>(n);
}

/// Combined stable substep count given both advection and diffusion limits.
inline std::size_t stable_substeps(double total_dt, double velocity, double dz,
                                   double diffusivity, double cfl_max = 0.5,
                                   double diff_max = 0.5)
{
  const double dt_adv = advection_dt_limit(velocity, dz, cfl_max);
  const double dt_diff = diffusion_dt_limit(diffusivity, dz, diff_max);
  const double dt_limit = std::min(dt_adv, dt_diff);
  return required_substeps(total_dt, dt_limit);
}

}  // namespace numerical
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__COMMON__NUMERICAL__CFL_CONTROL_HPP_
