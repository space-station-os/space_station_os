#include "ssos_eclss/wrs/multifiltration_model.hpp"

#include <algorithm>

namespace ssos_eclss
{
namespace wrs
{

MultifiltrationModel::MultifiltrationModel(const MultifiltrationParams & params)
: params_(params)
{}

void MultifiltrationModel::reset() { accumulated_kg_ = 0.0; }

MultifiltrationResult MultifiltrationModel::process(double dt, double feed_kg_s,
                                                    double feed_conductivity_us)
{
  MultifiltrationResult r{};
  const double cap = std::max(params_.bed_capacity_kg, 1.0e-12);
  const double utilization = accumulated_kg_ / cap;

  // Removal efficiency degrades as the bed approaches breakthrough.
  const double remaining = std::clamp(1.0 - utilization, 0.0, 1.0);
  const double eff = std::clamp(params_.removal_efficiency, 0.0, 1.0) * remaining;

  r.product_kg_s = feed_kg_s;
  r.product_conductivity_us = feed_conductivity_us * (1.0 - eff);
  // Contaminant mass captured ~ proportional to feed * conductivity * eff.
  // Use conductivity as a proxy for contaminant concentration [g per uS scale].
  const double captured_kg = feed_kg_s * feed_conductivity_us * 1.0e-6 * eff * dt;
  accumulated_kg_ += captured_kg;

  r.bed_utilization = std::clamp(accumulated_kg_ / cap, 0.0, 1.0);
  r.broken_through = (accumulated_kg_ >= cap);
  return r;
}

}  // namespace wrs
}  // namespace ssos_eclss
