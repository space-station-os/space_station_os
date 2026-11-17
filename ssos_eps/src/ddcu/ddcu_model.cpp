#include "ssos_eps/ddcu/ddcu_model.hpp"

#include <algorithm>
#include <cmath>

namespace ssos_eps
{
namespace ddcu
{

std::string to_string(DdcuMode mode)
{
  switch (mode) {
    case DdcuMode::OFF:
      return "off";
    case DdcuMode::REGULATING:
      return "regulating";
    case DdcuMode::OVERLOAD:
      return "overload";
    case DdcuMode::TRIPPED:
      return "tripped";
  }
  return "unknown";
}

DdcuModel::DdcuModel(const DdcuParams & params)
: params_(params)
{
  reset();
}

void DdcuModel::reset()
{
  state_ = DdcuState{};
  state_.mode = DdcuMode::OFF;
  state_.temperature_c = params_.initial_temp_c;
  faulted_ = false;
  over_temperature_ = false;
}

double DdcuModel::input_power_for(double output_w) const
{
  const double eta = std::clamp(params_.efficiency, 1.0e-6, 1.0);
  return std::max(output_w, 0.0) / eta + std::max(params_.no_load_loss_w, 0.0);
}

bool DdcuModel::available(double input_voltage_v) const
{
  return !faulted_ && !over_temperature_ &&
         input_voltage_v >= params_.input_min_v && input_voltage_v <= params_.input_max_v;
}

const DdcuState & DdcuModel::step(double dt, const DdcuInput & input)
{
  dt = std::max(dt, 0.0);
  const double eta = std::clamp(params_.efficiency, 1.0e-6, 1.0);
  const double rated = std::max(params_.rated_power_w, 0.0);

  state_.input_voltage_v = input.input_voltage_v;
  state_.requested_w = std::max(input.requested_w, 0.0);
  state_.faulted = faulted_;

  double output = 0.0;
  double drawn = 0.0;
  if (faulted_ || over_temperature_) {
    state_.mode = DdcuMode::TRIPPED;
  } else if (!available(input.input_voltage_v)) {
    state_.mode = DdcuMode::OFF;
  } else {
    output = std::min(std::max(input.demand_w, 0.0), rated);
    drawn = input_power_for(output);
    const double limit = std::max(input.input_limit_w, 0.0);
    if (drawn > limit) {
      // Brown-out: the primary side cannot deliver the full demand.
      drawn = limit;
      output = std::max((limit - params_.no_load_loss_w) * eta, 0.0);
    }
    state_.mode = state_.requested_w > rated ? DdcuMode::OVERLOAD : DdcuMode::REGULATING;
  }

  state_.output_power_w = output;
  state_.input_power_w = drawn;
  state_.heat_w = std::max(drawn - output, 0.0);
  state_.load_fraction = rated > 0.0 ? output / rated : 0.0;
  state_.output_voltage_v = state_.mode == DdcuMode::REGULATING ||
    state_.mode == DdcuMode::OVERLOAD ?
    params_.output_voltage_v - params_.regulation_droop_v * state_.load_fraction : 0.0;

  if (dt > 0.0 && params_.thermal_mass_j_k > 0.0) {
    const double g = std::max(params_.heat_transfer_w_k, 0.0);
    if (g > 0.0) {
      const double t_eq = params_.coldplate_temp_c + state_.heat_w / g;
      const double decay = std::exp(-g * dt / params_.thermal_mass_j_k);
      state_.temperature_c = t_eq + (state_.temperature_c - t_eq) * decay;
    } else {
      state_.temperature_c += state_.heat_w * dt / params_.thermal_mass_j_k;
    }
  }
  if (state_.temperature_c >= params_.trip_temp_c) {
    over_temperature_ = true;
  } else if (over_temperature_ && state_.temperature_c <= params_.reset_temp_c) {
    over_temperature_ = false;
  }
  state_.over_temperature = over_temperature_;

  switch (state_.mode) {
    case DdcuMode::TRIPPED:
      state_.status_message = faulted_ ? "DDCU fault: output isolated" :
        "DDCU over-temperature trip";
      break;
    case DdcuMode::OFF:
      state_.status_message = "primary input outside limits";
      break;
    case DdcuMode::OVERLOAD:
      state_.status_message = "requested load above rating";
      break;
    case DdcuMode::REGULATING:
      state_.status_message = output + 1.0e-6 < std::min(input.demand_w, rated) ?
        "primary supply limited" : "nominal";
      break;
  }
  return state_;
}

}  // namespace ddcu
}  // namespace ssos_eps
