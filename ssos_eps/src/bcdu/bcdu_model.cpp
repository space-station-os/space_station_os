#include "ssos_eps/bcdu/bcdu_model.hpp"

#include <algorithm>
#include <cmath>

namespace ssos_eps
{
namespace bcdu
{

std::string to_string(BcduMode mode)
{
  switch (mode) {
    case BcduMode::IDLE:
      return "idle";
    case BcduMode::CHARGING:
      return "charging";
    case BcduMode::DISCHARGING:
      return "discharging";
    case BcduMode::FAULT:
      return "fault";
  }
  return "unknown";
}

BcduModel::BcduModel(const BcduParams & params)
: params_(params)
{
  reset();
}

void BcduModel::reset()
{
  state_ = BcduState{};
  state_.mode = BcduMode::IDLE;
  state_.source_available = false;
  state_.regulation_voltage_v = params_.regulation_voltage_v;
  faulted_ = false;
}

double BcduModel::distribute(double dt, double power_w,
                             std::vector<battery::BatteryOruModel> & orus)
{
  std::vector<double> capability(orus.size(), 0.0);
  double total_capability = 0.0;
  for (std::size_t i = 0; i < orus.size(); ++i) {
    capability[i] = power_w > 0.0 ? orus[i].max_discharge_power_w(dt) :
      power_w < 0.0 ? orus[i].max_charge_power_w(dt) : 0.0;
    total_capability += capability[i];
  }

  const double magnitude = std::min(std::abs(power_w), total_capability);
  const double sign = power_w >= 0.0 ? 1.0 : -1.0;
  double exchanged = 0.0;
  for (std::size_t i = 0; i < orus.size(); ++i) {
    const double share = total_capability > 0.0 ?
      magnitude * capability[i] / total_capability : 0.0;
    exchanged += orus[i].step(dt, sign * share);
  }
  return exchanged;
}

const BcduState & BcduModel::step(double dt, const BcduInput & input,
                                  std::vector<battery::BatteryOruModel> & orus)
{
  const double regulation_v = std::clamp(params_.regulation_voltage_v,
                                         params_.regulation_min_v,
                                         params_.regulation_max_v);
  const double eta_c = std::clamp(params_.charge_efficiency, 1.0e-6, 1.0);
  const double eta_d = std::clamp(params_.discharge_efficiency, 1.0e-6, 1.0);
  const double load = std::max(input.load_power_w, 0.0);

  if (input.ssu_voltage_v >= params_.charge_enter_v) {
    state_.source_available = true;
  } else if (input.ssu_voltage_v <= params_.discharge_enter_v) {
    state_.source_available = false;
  }
  const double source = state_.source_available ? std::max(input.source_power_w, 0.0) : 0.0;

  state_.faulted = faulted_;
  state_.regulation_voltage_v = regulation_v;
  state_.failed_orus = 0;
  for (const auto & oru : orus) {
    if (oru.failed()) {
      ++state_.failed_orus;
    }
  }

  double battery_w = 0.0;
  double bus_w = 0.0;
  double served = std::min(load, source);

  if (faulted_) {
    distribute(dt, 0.0, orus);
    state_.mode = BcduMode::FAULT;
  } else if (source - load > params_.deadband_w) {
    const double surplus = source - load;
    const double bus_limit_w = params_.max_charge_current_a * input.ssu_voltage_v;
    const double request_w = std::min(surplus, std::max(bus_limit_w, 0.0)) * eta_c;
    battery_w = distribute(dt, -request_w, orus);
    bus_w = battery_w / eta_c;
    served = load;
    state_.mode = -battery_w > params_.deadband_w * eta_c ?
      BcduMode::CHARGING : BcduMode::IDLE;
  } else if (load - source > 0.0) {
    const double deficit = load - source;
    const double bus_limit_w = params_.max_discharge_current_a * regulation_v;
    const double request_w = std::min(deficit, std::max(bus_limit_w, 0.0)) / eta_d;
    battery_w = distribute(dt, request_w, orus);
    bus_w = battery_w * eta_d;
    served = source + bus_w;
    state_.mode = battery_w > 0.0 ? BcduMode::DISCHARGING : BcduMode::IDLE;
  } else {
    distribute(dt, 0.0, orus);
    state_.mode = BcduMode::IDLE;
  }

  state_.battery_power_w = battery_w;
  state_.bus_power_w = bus_w;
  state_.conversion_loss_w = std::abs(battery_w - bus_w);
  state_.load_served_w = std::min(served, load);
  state_.unserved_load_w = std::max(load - state_.load_served_w, 0.0);
  const double charge_draw_w = std::max(-bus_w, 0.0);
  state_.shunted_power_w = std::max(source - std::min(load, source) - charge_draw_w, 0.0);

  bool can_discharge = false;
  for (const auto & oru : orus) {
    if (!oru.failed() && oru.state().soc > 0.0) {
      can_discharge = true;
      break;
    }
  }
  if (source > 0.0) {
    state_.bus_voltage_v = input.ssu_voltage_v;
  } else if (!faulted_ && can_discharge) {
    state_.bus_voltage_v = regulation_v;
  } else {
    state_.bus_voltage_v = 0.0;
  }
  state_.current_draw_a = state_.bus_voltage_v > 0.0 ? bus_w / state_.bus_voltage_v : 0.0;

  state_.degraded = state_.unserved_load_w > params_.unserved_tolerance_w ||
    state_.failed_orus > 0;
  if (faulted_) {
    state_.status_message = "BCDU fault: batteries isolated";
  } else if (state_.unserved_load_w > params_.unserved_tolerance_w) {
    state_.status_message = "channel load not fully supplied";
  } else if (state_.failed_orus > 0) {
    state_.status_message = std::to_string(state_.failed_orus) + " battery ORU(s) failed";
  } else {
    state_.status_message = "nominal";
  }
  return state_;
}

}  // namespace bcdu
}  // namespace ssos_eps
