#include "ssos_eclss/common/heat_transfer.hpp"

#include <algorithm>
#include <cmath>

namespace ssos_eclss
{
namespace heat
{

double prandtl_number(double cp, double viscosity, double thermal_cond)
{
  if (thermal_cond <= 0.0) {
    return 0.0;
  }
  return cp * viscosity / thermal_cond;
}

double nusselt_wakao_kaguei(double reynolds, double prandtl)
{
  return 2.0 + 1.1 * std::pow(std::max(reynolds, 0.0), 0.6) *
                  std::cbrt(std::max(prandtl, 0.0));
}

double gas_solid_htc(double nusselt, double thermal_cond, double particle_d)
{
  if (particle_d <= 0.0) {
    return 0.0;
  }
  return nusselt * thermal_cond / particle_d;
}

double specific_surface_area(double voidage, double particle_d)
{
  const double e = std::clamp(voidage, 0.0, 0.999);
  if (particle_d <= 0.0) {
    return 0.0;
  }
  return 6.0 * (1.0 - e) / particle_d;
}

double ntu(double ua, double c_min)
{
  if (c_min <= 0.0) {
    return 0.0;
  }
  return ua / c_min;
}

double effectiveness_counterflow(double ntu_val, double cr)
{
  cr = std::clamp(cr, 0.0, 1.0);
  if (ntu_val <= 0.0) {
    return 0.0;
  }
  if (cr < 1.0e-6) {
    // Cr -> 0 (e.g. condensing/evaporating stream).
    return 1.0 - std::exp(-ntu_val);
  }
  if (std::abs(cr - 1.0) < 1.0e-6) {
    return ntu_val / (1.0 + ntu_val);
  }
  const double e = std::exp(-ntu_val * (1.0 - cr));
  return (1.0 - e) / (1.0 - cr * e);
}

double effectiveness_crossflow(double ntu_val, double cr)
{
  cr = std::clamp(cr, 0.0, 1.0);
  if (ntu_val <= 0.0) {
    return 0.0;
  }
  if (cr < 1.0e-6) {
    return 1.0 - std::exp(-ntu_val);
  }
  // Both fluids unmixed (approximate closed form).
  const double term = std::pow(ntu_val, 0.22) / cr *
                      (std::exp(-cr * std::pow(ntu_val, 0.78)) - 1.0);
  return 1.0 - std::exp(term);
}

double hx_heat_duty(double effectiveness, double c_min, double t_hot_in,
                    double t_cold_in)
{
  return effectiveness * c_min * (t_hot_in - t_cold_in);
}

}  // namespace heat
}  // namespace ssos_eclss
