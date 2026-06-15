#ifndef SSOS_ECLSS__SABATIER__SABATIER_PARAMETERS_HPP_
#define SSOS_ECLSS__SABATIER__SABATIER_PARAMETERS_HPP_

// Parameters for the Sabatier CO2 reduction reactor:
//   CO2 + 4 H2 -> CH4 + 2 H2O   (exothermic, dH ~ -165 kJ/mol CO2)
// SI units. Factory functions provide flight-class defaults.

namespace ssos_eclss
{
namespace sabatier
{

/// Reaction kinetics / equilibrium parameters.
///
/// The Sabatier reaction is kinetically limited at low temperature (slow rate
/// below ~375 C) and thermodynamically (equilibrium) limited at high
/// temperature (the exothermic reaction is favoured at low T). The achievable
/// conversion is therefore the MINIMUM of a kinetic limit (rising with T) and an
/// equilibrium limit (falling with T), peaking at an intermediate temperature.
/// Ref: Hintze et al., "Sabatier System Design Study for a Mars ISRU Propellant
/// Production Plant", ICES-2018-155.
struct KineticsParams
{
  double pre_exponential;       // Arrhenius pre-exponential [1/s]
  double activation_energy;     // Arrhenius activation energy [J/mol]
  double max_conversion;        // equilibrium ceiling at low T [-]
  double residence_time_s;      // gas residence time in the catalyst bed [s]
  double heat_of_reaction;      // exothermic heat released [J/mol CO2] (positive)
  double equilibrium_knee_temp_k;   // T above which equilibrium conversion declines [K]
  double equilibrium_decline_per_k; // equilibrium conversion lost per K above knee [1/K]
};

/// Reactor thermal / operating parameters.
struct ReactorParams
{
  double operating_temp_k;    // nominal catalyst temperature [K]
  double thermal_mass_j_k;    // lumped reactor thermal mass [J/K]
  double heat_loss_coeff_w_k; // reactor-to-ambient conductance [W/K]
  double ambient_temp_k;      // ambient temperature [K]
};

/// Complete Sabatier parameter set.
struct SabatierParameters
{
  KineticsParams kinetics;
  ReactorParams reactor;
};

KineticsParams default_kinetics_params();
ReactorParams default_reactor_params();
SabatierParameters default_sabatier_parameters();

}  // namespace sabatier
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__SABATIER__SABATIER_PARAMETERS_HPP_
