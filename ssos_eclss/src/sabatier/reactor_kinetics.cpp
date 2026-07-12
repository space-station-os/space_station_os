#include "ssos_eclss/sabatier/reactor_kinetics.hpp"

#include <algorithm>
#include <cmath>

#include "ssos_eclss/common/units.hpp"

namespace ssos_eclss
{
namespace sabatier
{

ReactorKinetics::ReactorKinetics(const KineticsParams & params) : params_(params) {}

double ReactorKinetics::rate_constant(double temperature_k) const
{
  if (temperature_k <= 0.0) {
    return 0.0;
  }
  return params_.pre_exponential *
         std::exp(-params_.activation_energy / (units::R_GAS * temperature_k));
}

double ReactorKinetics::kinetic_limit(double temperature_k) const
{
  const double k = rate_constant(temperature_k);
  return std::clamp(1.0 - std::exp(-k * params_.residence_time_s), 0.0, 1.0);
}

double ReactorKinetics::equilibrium_limit(double temperature_k) const
{
  double x_eq = params_.max_conversion;
  if (temperature_k > params_.equilibrium_knee_temp_k) {
    x_eq -= params_.equilibrium_decline_per_k *
            (temperature_k - params_.equilibrium_knee_temp_k);
  }
  return std::clamp(x_eq, 0.0, params_.max_conversion);
}

double ReactorKinetics::conversion(double temperature_k) const
{
  return std::min(kinetic_limit(temperature_k), equilibrium_limit(temperature_k));
}

}  // namespace sabatier
}  // namespace ssos_eclss
