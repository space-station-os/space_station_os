#ifndef SSOS_ECLSS__CABIN__LEAK_MODEL_HPP_
#define SSOS_ECLSS__CABIN__LEAK_MODEL_HPP_

#include "ssos_eclss/cabin/cabin_atmosphere.hpp"

// Cabin atmosphere leakage to vacuum. A small nominal structural leak plus a
// fault-injectable larger leak (e.g. a micrometeoroid puncture). Models choked
// orifice outflow proportional to cabin pressure and composition. No ROS.

namespace ssos_eclss
{
namespace cabin
{

/// Leak parameters.
struct LeakParams
{
  double nominal_area_m2;  // baseline equivalent leak orifice area [m^2]
  double discharge_coeff;  // orifice discharge coefficient [-]
};

/// Cabin leak model.
class LeakModel
{
public:
  explicit LeakModel(const LeakParams & params);

  /// Set an additional fault leak area [m^2] (0 to clear).
  void set_fault_area(double area_m2) { fault_area_m2_ = (area_m2 > 0.0) ? area_m2 : 0.0; }

  /// Total effective leak area [m^2].
  double effective_area() const { return params_.nominal_area_m2 + fault_area_m2_; }

  /// Total mass leak rate [kg/s] for the current atmosphere (choked flow to
  /// vacuum). Always non-negative.
  double mass_leak_rate(const CabinAtmosphere & atm) const;

  /// Molar leak flows [mol/s] (negative = leaving cabin), composition-weighted.
  GasFlows leak_flows(const CabinAtmosphere & atm) const;

  const LeakParams & params() const { return params_; }

private:
  LeakParams params_;
  double fault_area_m2_{0.0};
};

}  // namespace cabin
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__CABIN__LEAK_MODEL_HPP_
