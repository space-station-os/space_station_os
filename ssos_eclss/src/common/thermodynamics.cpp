#include "ssos_eclss/common/thermodynamics.hpp"

#include <cmath>

#include "ssos_eclss/common/units.hpp"

namespace ssos_eclss
{
namespace thermo
{

namespace
{
// Quadratic mass-specific cp fits cp(T) = a + b*T + c*T^2 [J/(kg*K)].
// Coefficients fitted to NIST/NASA data over ~250..600 K for ECLSS use.
struct CpFit
{
  double a;
  double b;
  double c;
  double molar_mass;
};

CpFit fit_for(Species s)
{
  switch (s) {
    case Species::AIR:       return {950.0, 0.16, 0.0, units::M_AIR};
    case Species::CO2:       return {600.0, 0.95, -3.0e-4, units::M_CO2};
    case Species::H2O_VAPOR: return {1820.0, 0.30, 0.0, units::M_H2O};
    case Species::O2:        return {880.0, 0.12, 0.0, units::M_O2};
    case Species::H2:        return {14200.0, 0.5, 0.0, units::M_H2};
    case Species::N2:        return {1010.0, 0.07, 0.0, units::M_N2};
    case Species::CH4:       return {1700.0, 3.0, 0.0, units::M_CH4};
  }
  return {1005.0, 0.0, 0.0, units::M_AIR};
}
}  // namespace

double cp_mass(Species species, double temperature_k)
{
  const CpFit f = fit_for(species);
  return f.a + f.b * temperature_k + f.c * temperature_k * temperature_k;
}

double cp_molar(Species species, double temperature_k)
{
  const CpFit f = fit_for(species);
  return cp_mass(species, temperature_k) * f.molar_mass;
}

double cv_mass(Species species, double temperature_k)
{
  const CpFit f = fit_for(species);
  const double r_specific = units::R_GAS / f.molar_mass;
  return cp_mass(species, temperature_k) - r_specific;
}

double gamma_ratio(Species species, double temperature_k)
{
  const double cp = cp_mass(species, temperature_k);
  const double cv = cv_mass(species, temperature_k);
  return (cv > 0.0) ? cp / cv : 1.4;
}

double sensible_enthalpy(Species species, double temperature_k, double reference_k)
{
  const CpFit f = fit_for(species);
  // Integral of (a + b*T + c*T^2) dT from reference_k to temperature_k.
  auto integral = [&](double t) {
    return f.a * t + 0.5 * f.b * t * t + (1.0 / 3.0) * f.c * t * t * t;
  };
  return integral(temperature_k) - integral(reference_k);
}

double latent_heat_water(double temperature_k)
{
  // Watson correlation referenced to L0 = 2.501e6 J/kg at 273.15 K,
  // critical temperature Tc = 647.096 K, exponent 0.38.
  const double tc = 647.096;
  const double l0 = 2.501e6;
  const double t_ref = 273.15;
  if (temperature_k >= tc) {
    return 0.0;
  }
  const double ratio = (tc - temperature_k) / (tc - t_ref);
  return l0 * std::pow(ratio, 0.38);
}

double cp_liquid_water(double temperature_k)
{
  // Mild curvature; minimum near 308 K. Stay close to 4180..4220.
  const double dt = temperature_k - 308.0;
  return 4180.0 + 4.0e-3 * dt * dt;
}

double cp_sorbent_solid(double /*temperature_k*/)
{
  // Representative zeolite/silica specific heat [J/(kg*K)].
  return 920.0;
}

}  // namespace thermo
}  // namespace ssos_eclss
