#ifndef SSOS_ECLSS__COMMON__GAS_PROPERTIES_HPP_
#define SSOS_ECLSS__COMMON__GAS_PROPERTIES_HPP_

#include <vector>

// Ideal-gas, gas-mixture and psychrometric relations. No ROS, no external deps.

namespace ssos_eclss
{
namespace gas
{

/// Ideal-gas density [kg/m^3].
/// rho = P * M / (R * T)
/// @param pressure_pa  absolute pressure [Pa]
/// @param temperature_k temperature [K]
/// @param molar_mass   molar mass [kg/mol]
double gas_density(double pressure_pa, double temperature_k, double molar_mass);

/// Molar concentration of an ideal gas [mol/m^3] given partial pressure.
/// c = P / (R * T)
double concentration_from_pp(double partial_pressure_pa, double temperature_k);

/// Partial pressure [Pa] from molar concentration [mol/m^3].
/// P = c * R * T
double partial_pressure(double concentration_mol_m3, double temperature_k);

/// Saturation vapour pressure of water [Pa] using the Magnus/Tetens formula.
/// Valid roughly -45..60 degC.
/// @param temperature_k temperature [K]
double water_saturation_pressure(double temperature_k);

/// Dew point temperature [K] for a given water-vapour partial pressure [Pa].
/// Inverse of water_saturation_pressure (Magnus inversion).
double dew_point_from_pp(double water_pp_pa);

/// Relative humidity [-] (0..1+) from water vapour partial pressure and temperature.
double relative_humidity(double water_pp_pa, double temperature_k);

/// Water vapour partial pressure [Pa] from relative humidity and temperature.
double water_pp_from_rh(double relative_humidity, double temperature_k);

/// Absolute humidity (humidity ratio) [kg water / kg dry air].
/// @param water_pp_pa  water vapour partial pressure [Pa]
/// @param total_pressure_pa total pressure [Pa]
double humidity_ratio(double water_pp_pa, double total_pressure_pa);

/// Dynamic viscosity of a single gas [Pa*s] via Sutherland's law.
/// Defaults correspond to air.
double sutherland_viscosity(double temperature_k,
                            double mu_ref = 1.716e-5,
                            double t_ref = 273.15,
                            double sutherland_c = 111.0);

/// One component of a gas mixture: mole fraction, molar mass and viscosity.
struct MixtureComponent
{
  double mole_fraction;  // [-]
  double molar_mass;     // [kg/mol]
  double viscosity;      // [Pa*s]
};

/// Mixture dynamic viscosity [Pa*s] via Wilke's mixing rule.
double mixture_viscosity(const std::vector<MixtureComponent> & components);

/// Mixture molar mass [kg/mol] (mole-fraction weighted).
double mixture_molar_mass(const std::vector<MixtureComponent> & components);

}  // namespace gas
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__COMMON__GAS_PROPERTIES_HPP_
