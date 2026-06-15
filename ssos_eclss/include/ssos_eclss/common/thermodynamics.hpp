#ifndef SSOS_ECLSS__COMMON__THERMODYNAMICS_HPP_
#define SSOS_ECLSS__COMMON__THERMODYNAMICS_HPP_

// Heat capacities, enthalpy and phase-change relations. No ROS, no external deps.

namespace ssos_eclss
{
namespace thermo
{

/// Identifies a gas species for property lookups.
enum class Species
{
  AIR,
  CO2,
  H2O_VAPOR,
  O2,
  H2,
  N2,
  CH4
};

/// Specific heat at constant pressure [J/(kg*K)] using a Shomate/NASA-style
/// quadratic fit cp(T) = a + b*T + c*T^2. Falls back to a constant for species
/// with weak temperature dependence over ECLSS-relevant ranges.
/// @param species gas species
/// @param temperature_k temperature [K]
double cp_mass(Species species, double temperature_k);

/// Molar specific heat at constant pressure [J/(mol*K)].
double cp_molar(Species species, double temperature_k);

/// Specific heat at constant volume [J/(kg*K)], cv = cp - R/M (ideal gas).
double cv_mass(Species species, double temperature_k);

/// Ratio of specific heats gamma = cp/cv [-].
double gamma_ratio(Species species, double temperature_k);

/// Sensible enthalpy [J/kg] relative to a reference temperature (default 298.15 K),
/// integral of cp(T) dT.
double sensible_enthalpy(Species species, double temperature_k,
                         double reference_k = 298.15);

/// Latent heat of vaporisation of water [J/kg] as a function of temperature.
/// Watson correlation, valid 273..473 K.
double latent_heat_water(double temperature_k);

/// Specific heat of liquid water [J/(kg*K)] (weak T dependence, near 4186).
double cp_liquid_water(double temperature_k);

/// Specific heat of a packed-bed solid sorbent [J/(kg*K)].
/// Representative value for zeolite / silica adsorbents.
double cp_sorbent_solid(double temperature_k);

}  // namespace thermo
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__COMMON__THERMODYNAMICS_HPP_
