#include "ssos_eclss/common/gas_properties.hpp"

#include <cmath>

#include "ssos_eclss/common/units.hpp"

namespace ssos_eclss
{
namespace gas
{

double gas_density(double pressure_pa, double temperature_k, double molar_mass)
{
  return pressure_pa * molar_mass / (units::R_GAS * temperature_k);
}

double concentration_from_pp(double partial_pressure_pa, double temperature_k)
{
  return partial_pressure_pa / (units::R_GAS * temperature_k);
}

double partial_pressure(double concentration_mol_m3, double temperature_k)
{
  return concentration_mol_m3 * units::R_GAS * temperature_k;
}

double water_saturation_pressure(double temperature_k)
{
  // Magnus/Tetens: Psat[Pa] = 610.94 * exp(17.625 * Tc / (Tc + 243.04))
  const double tc = units::kelvin_to_celsius(temperature_k);
  return 610.94 * std::exp(17.625 * tc / (tc + 243.04));
}

double dew_point_from_pp(double water_pp_pa)
{
  // Invert Magnus. Guard against non-positive pressure.
  if (water_pp_pa <= 0.0) {
    return 0.0;  // 0 K sentinel; caller treats as "bone dry"
  }
  const double ln_ratio = std::log(water_pp_pa / 610.94);
  const double tc = 243.04 * ln_ratio / (17.625 - ln_ratio);
  return units::celsius_to_kelvin(tc);
}

double relative_humidity(double water_pp_pa, double temperature_k)
{
  const double psat = water_saturation_pressure(temperature_k);
  if (psat <= 0.0) {
    return 0.0;
  }
  return water_pp_pa / psat;
}

double water_pp_from_rh(double relative_humidity, double temperature_k)
{
  return relative_humidity * water_saturation_pressure(temperature_k);
}

double humidity_ratio(double water_pp_pa, double total_pressure_pa)
{
  const double dry_pp = total_pressure_pa - water_pp_pa;
  if (dry_pp <= 0.0) {
    return 0.0;
  }
  // w = (M_w / M_air) * Pw / (Ptot - Pw)
  return (units::M_H2O / units::M_AIR) * water_pp_pa / dry_pp;
}

double sutherland_viscosity(double temperature_k, double mu_ref, double t_ref,
                            double sutherland_c)
{
  const double ratio = temperature_k / t_ref;
  return mu_ref * std::pow(ratio, 1.5) * (t_ref + sutherland_c) /
         (temperature_k + sutherland_c);
}

double mixture_viscosity(const std::vector<MixtureComponent> & components)
{
  // Wilke's semi-empirical mixing rule.
  const std::size_t n = components.size();
  if (n == 0) {
    return 0.0;
  }
  double mu_mix = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    double denom = 0.0;
    for (std::size_t j = 0; j < n; ++j) {
      const double mu_ratio = components[i].viscosity / components[j].viscosity;
      const double m_ratio = components[j].molar_mass / components[i].molar_mass;
      const double num = 1.0 + std::sqrt(mu_ratio) * std::pow(m_ratio, 0.25);
      const double phi = num * num /
                         std::sqrt(8.0 * (1.0 + components[i].molar_mass /
                                          components[j].molar_mass));
      denom += components[j].mole_fraction * phi;
    }
    if (denom > 0.0) {
      mu_mix += components[i].mole_fraction * components[i].viscosity / denom;
    }
  }
  return mu_mix;
}

double mixture_molar_mass(const std::vector<MixtureComponent> & components)
{
  double m = 0.0;
  double total_fraction = 0.0;
  for (const auto & c : components) {
    m += c.mole_fraction * c.molar_mass;
    total_fraction += c.mole_fraction;
  }
  if (total_fraction > 0.0) {
    m /= total_fraction;  // normalise in case fractions don't sum to 1
  }
  return m;
}

}  // namespace gas
}  // namespace ssos_eclss
