#include "ssos_eps/system/eps_system.hpp"

#include <algorithm>
#include <limits>

namespace ssos_eps
{
namespace system
{

EpsSystem::EpsSystem(const EpsParameters & params)
: params_(params),
  array_(params.solar.array, params.solar.thermal),
  ssu_(params.solar.ssu)
{
  reset();
}

void EpsSystem::reset()
{
  const int n_channels = std::max(params_.n_channels, 1);
  const int per_channel = std::max(params_.orus_per_channel, 1);

  array_.set_params(params_.solar.array, params_.solar.thermal);
  array_.reset();
  ssu_.set_params(params_.solar.ssu);
  ssu_.reset();

  bcdus_.assign(static_cast<std::size_t>(n_channels), bcdu::BcduModel(params_.bcdu));
  orus_by_channel_.assign(
    static_cast<std::size_t>(n_channels),
    std::vector<battery::BatteryOruModel>(
      static_cast<std::size_t>(per_channel), battery::BatteryOruModel(params_.battery)));

  orus_.clear();
  for (auto & channel : orus_by_channel_) {
    for (auto & oru : channel) {
      orus_.push_back(&oru);
    }
  }

  state_ = EpsState{};
  state_.channels.resize(bcdus_.size());
  state_.orus.resize(orus_.size());
  for (std::size_t i = 0; i < bcdus_.size(); ++i) {
    state_.channels[i] = bcdus_[i].state();
  }
  for (std::size_t i = 0; i < orus_.size(); ++i) {
    state_.orus[i] = orus_[i]->state();
  }
  summarise(EpsInput{0.0, true, 0.0});
}

void EpsSystem::set_parameters(const EpsParameters & params)
{
  const bool topology_changed = params.n_channels != params_.n_channels ||
    params.orus_per_channel != params_.orus_per_channel;
  params_ = params;
  if (topology_changed) {
    reset();
    return;
  }
  array_.set_params(params_.solar.array, params_.solar.thermal);
  ssu_.set_params(params_.solar.ssu);
  for (auto & b : bcdus_) {
    b.set_params(params_.bcdu);
  }
  for (auto * oru : orus_) {
    oru->set_params(params_.battery);
  }
}

void EpsSystem::set_oru_failed(int index, bool failed)
{
  if (index >= 0 && index < n_orus()) {
    orus_[static_cast<std::size_t>(index)]->set_failed(failed);
  }
}

void EpsSystem::set_bcdu_fault(int channel, bool faulted)
{
  if (channel >= 0 && channel < n_channels()) {
    bcdus_[static_cast<std::size_t>(channel)].set_fault(faulted);
  }
}

void EpsSystem::set_array_degradation(double factor)
{
  params_.solar.array.degradation_factor = std::clamp(factor, 0.0, 1.0);
  array_.set_params(params_.solar.array, params_.solar.thermal);
}

const EpsState & EpsSystem::step(double dt, const EpsInput & input)
{
  dt = std::max(dt, 0.0);
  state_.array = array_.step(dt, input.solar_flux_w_m2, input.in_eclipse);
  state_.ssu = ssu_.step(dt, state_.array.array_power_w);

  const double n = static_cast<double>(bcdus_.size());
  bcdu::BcduInput channel_input{};
  channel_input.ssu_voltage_v = state_.ssu.bus_voltage_v;
  channel_input.source_power_w = state_.ssu.available_power_w / n;
  channel_input.load_power_w = std::max(input.load_demand_w, 0.0) / n;

  for (std::size_t c = 0; c < bcdus_.size(); ++c) {
    state_.channels[c] = bcdus_[c].step(dt, channel_input, orus_by_channel_[c]);
  }
  for (std::size_t i = 0; i < orus_.size(); ++i) {
    state_.orus[i] = orus_[i]->state();
  }
  summarise(input);
  return state_;
}

void EpsSystem::summarise(const EpsInput & input)
{
  state_.generation_w = state_.array.array_power_w;
  state_.available_power_w = state_.ssu.available_power_w;
  state_.load_demand_w = std::max(input.load_demand_w, 0.0);
  state_.power_balance_w = state_.available_power_w - state_.load_demand_w;

  state_.load_served_w = 0.0;
  state_.unserved_load_w = 0.0;
  state_.battery_power_w = 0.0;
  state_.shunted_power_w = 0.0;
  state_.conversion_loss_w = 0.0;
  state_.bcdu_fault = false;
  for (const auto & ch : state_.channels) {
    state_.load_served_w += ch.load_served_w;
    state_.unserved_load_w += ch.unserved_load_w;
    state_.battery_power_w += ch.battery_power_w;
    state_.shunted_power_w += ch.shunted_power_w;
    state_.conversion_loss_w += ch.conversion_loss_w;
    state_.bcdu_fault = state_.bcdu_fault || ch.faulted;
  }

  state_.battery_charge_w = 0.0;
  state_.battery_discharge_w = 0.0;
  state_.stored_energy_wh = 0.0;
  state_.oru_failed = false;
  state_.min_oru_soc = state_.orus.empty() ? 0.0 : std::numeric_limits<double>::max();
  double weighted_soc = 0.0;
  double total_capacity = 0.0;
  for (std::size_t i = 0; i < state_.orus.size(); ++i) {
    const auto & s = state_.orus[i];
    state_.battery_charge_w += std::max(-s.power_w, 0.0);
    state_.battery_discharge_w += std::max(s.power_w, 0.0);
    state_.stored_energy_wh += s.stored_energy_wh;
    state_.min_oru_soc = std::min(state_.min_oru_soc, s.soc);
    state_.oru_failed = state_.oru_failed || s.health == battery::BatteryHealth::FAILED;
    const double capacity = orus_[i]->usable_capacity_ah();
    weighted_soc += s.soc * capacity;
    total_capacity += capacity;
  }
  state_.battery_soc = total_capacity > 0.0 ? weighted_soc / total_capacity : 0.0;

  const double tolerance = params_.bcdu.unserved_tolerance_w;
  state_.low_soc = state_.battery_soc < params_.low_soc_threshold;
  state_.load_unserved = state_.unserved_load_w > tolerance;
  state_.healthy = !state_.low_soc && !state_.load_unserved &&
    !state_.bcdu_fault && !state_.oru_failed;

  if (state_.load_unserved) {
    state_.status_message = "station load not fully supplied";
  } else if (state_.bcdu_fault) {
    state_.status_message = "BCDU fault";
  } else if (state_.low_soc) {
    state_.status_message = "battery state of charge below threshold";
  } else if (state_.oru_failed) {
    state_.status_message = "battery ORU failed";
  } else {
    state_.status_message = "nominal";
  }
}

}  // namespace system
}  // namespace ssos_eps
