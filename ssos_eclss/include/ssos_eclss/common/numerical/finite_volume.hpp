#ifndef SSOS_ECLSS__COMMON__NUMERICAL__FINITE_VOLUME_HPP_
#define SSOS_ECLSS__COMMON__NUMERICAL__FINITE_VOLUME_HPP_

#include <cstddef>
#include <vector>

// 1D finite-volume discretisation helpers for the packed-bed PDE solver.
// First-order upwind advection and central-difference diffusion on a uniform
// grid. Header-only, no dependencies.

namespace ssos_eclss
{
namespace numerical
{

/// Upwind advection derivative -v * d(phi)/dz for a uniform 1D grid.
/// Boundary handling: cell 0 uses @p inlet_value as the upstream ghost value
/// when velocity is positive; the last cell uses a zero-gradient (outflow)
/// condition. For negative velocity the roles reverse.
/// @param phi   cell-centred field (size N)
/// @param velocity advection velocity [m/s] (sign sets the upwind direction)
/// @param dz    cell size [m]
/// @param inlet_value upstream boundary value of phi
/// @param out   output derivative, resized to N
inline void upwind_advection(const std::vector<double> & phi, double velocity,
                             double dz, double inlet_value,
                             std::vector<double> & out)
{
  const std::size_t n = phi.size();
  out.assign(n, 0.0);
  if (n == 0 || dz <= 0.0) {
    return;
  }
  if (velocity >= 0.0) {
    for (std::size_t i = 0; i < n; ++i) {
      const double up = (i == 0) ? inlet_value : phi[i - 1];
      out[i] = -velocity * (phi[i] - up) / dz;
    }
  } else {
    for (std::size_t i = 0; i < n; ++i) {
      const double down = (i + 1 < n) ? phi[i + 1] : phi[i];  // outflow
      out[i] = -velocity * (down - phi[i]) / dz;
    }
  }
}

/// Central-difference diffusion derivative D * d^2(phi)/dz^2 on a uniform grid
/// with zero-gradient (Neumann) boundaries.
/// @param phi field (size N)
/// @param diffusivity D [m^2/s] (or k/(rho*cp) equivalent)
/// @param dz  cell size [m]
/// @param out output derivative, resized to N
inline void central_diffusion(const std::vector<double> & phi, double diffusivity,
                              double dz, std::vector<double> & out)
{
  const std::size_t n = phi.size();
  out.assign(n, 0.0);
  if (n < 2 || dz <= 0.0) {
    return;
  }
  const double inv_dz2 = 1.0 / (dz * dz);
  for (std::size_t i = 0; i < n; ++i) {
    const double left = (i == 0) ? phi[i] : phi[i - 1];          // Neumann
    const double right = (i + 1 < n) ? phi[i + 1] : phi[i];      // Neumann
    out[i] = diffusivity * (left - 2.0 * phi[i] + right) * inv_dz2;
  }
}

/// Uniform 1D grid descriptor.
struct Grid1D
{
  std::size_t n_cells{1};
  double length{1.0};

  /// Cell size dz [m].
  double dz() const { return (n_cells > 0) ? length / static_cast<double>(n_cells) : length; }

  /// Centre coordinate of cell i [m].
  double cell_center(std::size_t i) const { return (static_cast<double>(i) + 0.5) * dz(); }
};

}  // namespace numerical
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__COMMON__NUMERICAL__FINITE_VOLUME_HPP_
