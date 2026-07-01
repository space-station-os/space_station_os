#ifndef SSOS_ECLSS__ARS__ADSORPTION_ISOTHERM_HPP_
#define SSOS_ECLSS__ARS__ADSORPTION_ISOTHERM_HPP_

#include "ssos_eclss/ars/ars_parameters.hpp"

// Equilibrium adsorption isotherms for the 4BMS sorbents. Toth single-component
// form plus competitive CO2/H2O loading via the extended (multicomponent) Toth
// model. Provides analytic derivatives needed by the LDF / bed solver Jacobian.
// No ROS, no external deps.

namespace ssos_eclss
{
namespace ars
{

/// Single-component Toth isotherm evaluator.
class TothIsotherm
{
public:
  explicit TothIsotherm(const TothParams & params);

  /// Temperature-dependent saturation loading q_m(T) [mol/kg].
  double q_max(double temperature_k) const;
  /// Temperature-dependent affinity b(T) [1/Pa].
  double affinity(double temperature_k) const;
  /// Temperature-dependent heterogeneity exponent t(T) [-], clamped to (0,1].
  double exponent(double temperature_k) const;

  /// Equilibrium loading q*(P,T) [mol/kg].
  /// @param partial_pressure_pa species partial pressure [Pa]
  /// @param temperature_k temperature [K]
  double loading(double partial_pressure_pa, double temperature_k) const;

  /// Partial derivative dq*/dP [mol/(kg*Pa)] (analytic).
  double dloading_dp(double partial_pressure_pa, double temperature_k) const;

  /// Partial derivative dq*/dT [mol/(kg*K)] (numerical central difference).
  double dloading_dt(double partial_pressure_pa, double temperature_k) const;

  /// Isosteric heat of adsorption [J/mol] (positive, exothermic).
  double heat_of_adsorption() const { return params_.dH; }

  const TothParams & params() const { return params_; }

private:
  TothParams params_;
};

/// Result of a competitive (multicomponent) equilibrium calculation.
struct CompetitiveLoading
{
  double q_co2;  // CO2 loading [mol/kg]
  double q_h2o;  // H2O loading [mol/kg]
};

/// Competitive CO2/H2O loading on 13X via the extended Toth model. Water
/// strongly suppresses CO2 capacity, which is why the desiccant beds dry the
/// air upstream of the CO2 beds. The denominator couples both adsorbates.
/// @param co2  CO2 Toth isotherm
/// @param h2o  H2O Toth isotherm
/// @param pp_co2_pa CO2 partial pressure [Pa]
/// @param pp_h2o_pa H2O partial pressure [Pa]
/// @param temperature_k temperature [K]
CompetitiveLoading competitive_loading(const TothIsotherm & co2,
                                       const TothIsotherm & h2o,
                                       double pp_co2_pa, double pp_h2o_pa,
                                       double temperature_k);

}  // namespace ars
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__ARS__ADSORPTION_ISOTHERM_HPP_
