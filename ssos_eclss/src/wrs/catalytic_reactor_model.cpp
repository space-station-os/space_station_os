#include "ssos_eclss/wrs/catalytic_reactor_model.hpp"

#include <algorithm>
#include <cmath>

namespace ssos_eclss
{
namespace wrs
{

CatalyticReactorModel::CatalyticReactorModel(const CatalyticReactorParams & params)
: params_(params)
{}

double CatalyticReactorModel::conversion(double temperature_k) const
{
  // Conversion approaches max_conversion as T rises above the activation
  // temperature: X = Xmax * (1 - exp(-(T - T0)/scale)) for T > T0.
  if (temperature_k <= params_.activation_temp_k) {
    return 0.0;
  }
  const double scale = 0.25 * params_.activation_temp_k;
  const double x = params_.max_conversion *
                   (1.0 - std::exp(-(temperature_k - params_.activation_temp_k) / scale));
  return std::clamp(x, 0.0, params_.max_conversion);
}

CatalyticResult CatalyticReactorModel::process(double voc_in_kg_s,
                                               double temperature_k) const
{
  CatalyticResult r{};
  r.conversion = conversion(temperature_k);
  r.outlet_voc_kg_s = std::max(0.0, voc_in_kg_s * (1.0 - r.conversion));
  return r;
}

}  // namespace wrs
}  // namespace ssos_eclss
