#ifndef SSOS_ECLSS__SABATIER__REACTOR_KINETICS_HPP_
#define SSOS_ECLSS__SABATIER__REACTOR_KINETICS_HPP_

#include "ssos_eclss/sabatier/sabatier_parameters.hpp"

// Sabatier reaction kinetics: CO2 + 4 H2 -> CH4 + 2 H2O. Arrhenius rate with an
// equilibrium-limited maximum conversion. No ROS, no external deps.

namespace ssos_eclss
{
namespace sabatier
{

/// Sabatier reaction kinetics.
class ReactorKinetics
{
public:
  explicit ReactorKinetics(const KineticsParams & params);

  /// Arrhenius rate constant k(T) [1/s].
  double rate_constant(double temperature_k) const;

  /// Kinetically-achievable conversion at temperature T over the catalyst
  /// residence time: X_kin = 1 - exp(-k(T) * tau). Rises with temperature.
  double kinetic_limit(double temperature_k) const;

  /// Thermodynamic (equilibrium) conversion ceiling. High (max_conversion) at
  /// low T, declining above the equilibrium knee temperature because the
  /// exothermic reaction is disfavoured at high T.
  double equilibrium_limit(double temperature_k) const;

  /// Actual fractional CO2 conversion = min(kinetic_limit, equilibrium_limit).
  /// Peaks at an intermediate temperature, matching the ISS Sabatier behaviour
  /// (kinetic limit below ~375 C, equilibrium limit at high T).
  double conversion(double temperature_k) const;

  const KineticsParams & params() const { return params_; }
  void set_params(const KineticsParams & p) { params_ = p; }

private:
  KineticsParams params_;
};

}  // namespace sabatier
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__SABATIER__REACTOR_KINETICS_HPP_
