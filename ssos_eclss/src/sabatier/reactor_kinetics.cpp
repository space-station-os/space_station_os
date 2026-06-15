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

double ReactorKinetics::conversion(double temperature_k) const
{
  const double k = rate_constant(temperature_k);
  const double x = 1.0 - std::exp(-k * params_.residence_time_s);
  return std::clamp(params_.max_conversion * x, 0.0, params_.max_conversion);
}

}  // namespace sabatier
}  // namespace ssos_eclss
