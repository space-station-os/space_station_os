#include "ssos_thermal/coolant/coolant_loop.hpp"

#include <algorithm>

namespace ssos_thermal
{
namespace coolant
{

CoolantLoop::CoolantLoop(const CoolantParams & params)
: params_(params)
{
}

CoolStepResult CoolantLoop::step(double node_temp_c, double target_temp_c) const
{
  const double delta_t = std::min(2.5, node_temp_c - target_temp_c);
  const double q_joules = params_.mass_kg * params_.specific_heat_j_per_kg_c * delta_t;
  const double q_kj = q_joules / 1000.0;

  const double new_temp = node_temp_c - delta_t;
  const double ammonia_heat_kj = q_kj * params_.heat_transfer_efficiency;
  // Dummy model, matching the legacy comment verbatim -- not a validated
  // ammonia thermodynamic relation, just enough to drive the feedback.
  const double ammonia_temp_c = 5.0 + (ammonia_heat_kj / 1000.0);

  CoolStepResult result;
  result.node_temp_c = new_temp;
  result.ammonia_temp_c = ammonia_temp_c;
  result.ammonia_heat_kj = ammonia_heat_kj;
  result.vent_triggered = ammonia_heat_kj >= params_.vent_threshold_kj;
  result.done = new_temp <= target_temp_c + 0.5;
  return result;
}

}  // namespace coolant
}  // namespace ssos_thermal
