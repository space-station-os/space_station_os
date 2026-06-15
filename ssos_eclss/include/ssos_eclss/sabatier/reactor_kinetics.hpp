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

  /// Fractional CO2 conversion at temperature T over the catalyst residence
  /// time, capped at the equilibrium-limited maximum. Uses first-order kinetics
  /// X = Xmax (1 - exp(-k(T) * tau)).
  double conversion(double temperature_k) const;

  const KineticsParams & params() const { return params_; }
  void set_params(const KineticsParams & p) { params_ = p; }

private:
  KineticsParams params_;
};

}  // namespace sabatier
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__SABATIER__REACTOR_KINETICS_HPP_
