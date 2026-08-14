#include "ssos_eclss/common/fluid_dynamics.hpp"

#include <algorithm>
#include <cmath>

namespace ssos_eclss
{
namespace fluid
{

double reynolds_number(double density, double velocity, double particle_d,
                       double viscosity)
{
  if (viscosity <= 0.0) {
    return 0.0;
  }
  return density * std::abs(velocity) * particle_d / viscosity;
}

double ergun_pressure_gradient(double velocity, double voidage, double particle_d,
                               double density, double viscosity)
{
  const double e = std::clamp(voidage, 1.0e-3, 0.999);
  const double e3 = e * e * e;
  const double one_minus_e = 1.0 - e;
  const double dp = particle_d;
  const double dp2 = dp * dp;

  // Viscous (Blake-Kozeny) term, linear in velocity.
  const double viscous = 150.0 * one_minus_e * one_minus_e / e3 *
                         viscosity * velocity / dp2;
  // Inertial (Burke-Plummer) term, quadratic; preserve sign of velocity.
  const double inertial = 1.75 * one_minus_e / e3 *
                          density * std::abs(velocity) * velocity / dp;
  return viscous + inertial;
}

double ergun_pressure_drop(double velocity, double voidage, double particle_d,
                           double density, double viscosity, double bed_length)
{
  return ergun_pressure_gradient(velocity, voidage, particle_d, density, viscosity) *
         bed_length;
}

double darcy_friction_factor(double reynolds, double relative_rough)
{
  if (reynolds <= 0.0) {
    return 0.0;
  }
  if (reynolds < 2300.0) {
    return 64.0 / reynolds;
  }
  // Haaland explicit approximation to Colebrook.
  const double term = std::pow(relative_rough / 3.7, 1.11) + 6.9 / reynolds;
  const double inv_sqrt_f = -1.8 * std::log10(term);
  return 1.0 / (inv_sqrt_f * inv_sqrt_f);
}

double superficial_velocity(double volumetric_flow_m3s, double cross_area_m2)
{
  if (cross_area_m2 <= 0.0) {
    return 0.0;
  }
  return volumetric_flow_m3s / cross_area_m2;
}

double interstitial_velocity(double superficial_v, double voidage)
{
  const double e = std::clamp(voidage, 1.0e-3, 0.999);
  return superficial_v / e;
}

}  // namespace fluid
}  // namespace ssos_eclss
