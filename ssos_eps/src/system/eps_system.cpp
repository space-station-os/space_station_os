#include "ssos_eps/system/eps_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ssos_eps
{
namespace system
{

namespace
{
constexpr double kPowerEpsilonW = 1.0e-6;
}  // namespace

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
  const int n_mbsu = std::max(params_.n_mbsu, 0);

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

  // Missing or out-of-range wiring leaves a channel or DDCU unconnected.
  params_.channel_mbsu.resize(static_cast<std::size_t>(n_channels), -1);
  for (auto & m : params_.channel_mbsu) {
    if (m < 0 || m >= n_mbsu) {
      m = -1;
    }
  }
  for (auto & m : params_.ddcu_mbsu) {
    if (m < 0 || m >= n_mbsu) {
      m = -1;
    }
  }
  mbsus_.assign(static_cast<std::size_t>(n_mbsu), mbsu::MbsuModel(params_.mbsu));
  ddcus_.assign(params_.ddcu_mbsu.size(), ddcu::DdcuModel(params_.ddcu));
  channel_offline_.assign(static_cast<std::size_t>(n_channels), false);

  registry_.clear();
  registry_.set_demand_scale(params_.demand_scale);
  load_baseline();

  state_ = EpsState{};
  state_.channels.resize(bcdus_.size());
  state_.orus.resize(orus_.size());
  state_.mbsus.resize(mbsus_.size());
  state_.ddcus.resize(ddcus_.size());
  for (std::size_t i = 0; i < bcdus_.size(); ++i) {
    state_.channels[i] = bcdus_[i].state();
  }
  for (std::size_t i = 0; i < orus_.size(); ++i) {
    state_.orus[i] = orus_[i]->state();
  }
  for (std::size_t i = 0; i < mbsus_.size(); ++i) {
    state_.mbsus[i] = mbsus_[i].state();
  }
  for (std::size_t i = 0; i < ddcus_.size(); ++i) {
    state_.ddcus[i] = ddcus_[i].state();
  }
  stepped_ = false;
  summarise();
}

void EpsSystem::load_baseline()
{
  for (auto spec : params_.baseline_loads) {
    if (spec.ddcu < 0 || spec.ddcu >= n_ddcu()) {
      int best = -1;
      double best_headroom = -std::numeric_limits<double>::infinity();
      for (int d = 0; d < n_ddcu(); ++d) {
        const double headroom = params_.ddcu.rated_power_w - registry_.requested_on_ddcu(d);
        if (headroom > best_headroom) {
          best = d;
          best_headroom = headroom;
        }
      }
      spec.ddcu = best;
    }
    registry_.upsert(spec);
  }
}

void EpsSystem::set_parameters(const EpsParameters & params)
{
  const bool topology_changed = params.n_channels != params_.n_channels ||
    params.orus_per_channel != params_.orus_per_channel ||
    params.n_mbsu != params_.n_mbsu ||
    params.channel_mbsu != params_.channel_mbsu ||
    params.ddcu_mbsu != params_.ddcu_mbsu;
  if (topology_changed) {
    params_ = params;
    reset();
    return;
  }
  const auto baseline = params_.baseline_loads;
  params_ = params;
  params_.baseline_loads = baseline;
  array_.set_params(params_.solar.array, params_.solar.thermal);
  ssu_.set_params(params_.solar.ssu);
  for (auto & b : bcdus_) {
    b.set_params(params_.bcdu);
  }
  for (auto * oru : orus_) {
    oru->set_params(params_.battery);
  }
  for (auto & m : mbsus_) {
    m.set_params(params_.mbsu);
  }
  for (auto & d : ddcus_) {
    d.set_params(params_.ddcu);
  }
  registry_.set_demand_scale(params_.demand_scale);
}

void EpsSystem::set_demand_scale(double scale)
{
  params_.demand_scale = std::max(scale, 0.0);
  registry_.set_demand_scale(params_.demand_scale);
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

void EpsSystem::set_battery_soc(double soc)
{
  for (std::size_t i = 0; i < orus_.size(); ++i) {
    orus_[i]->set_soc(soc);
    state_.orus[i] = orus_[i]->state();
  }
  summarise();
}

void EpsSystem::set_ddcu_fault(int index, bool faulted)
{
  if (index >= 0 && index < n_ddcu()) {
    ddcus_[static_cast<std::size_t>(index)].set_fault(faulted);
  }
}

void EpsSystem::set_mbsu_fault(int index, bool faulted)
{
  if (index >= 0 && index < n_mbsu()) {
    mbsus_[static_cast<std::size_t>(index)].set_fault(faulted);
  }
}

void EpsSystem::set_channel_offline(int channel, bool offline)
{
  if (channel >= 0 && channel < n_channels()) {
    channel_offline_[static_cast<std::size_t>(channel)] = offline;
  }
}

void EpsSystem::set_oru_extra_heat(int index, double heat_w)
{
  if (index >= 0 && index < n_orus()) {
    orus_[static_cast<std::size_t>(index)]->set_extra_heat_w(heat_w);
  }
}

void EpsSystem::set_ddcu_extra_heat(int index, double heat_w)
{
  if (index >= 0 && index < n_ddcu()) {
    ddcus_[static_cast<std::size_t>(index)].set_extra_heat_w(heat_w);
  }
}

void EpsSystem::set_gimbal_lock(bool locked)
{
  array_.set_gimbal_lock(locked);
}

int EpsSystem::mbsu_of_ddcu(int d) const
{
  return d >= 0 && d < n_ddcu() ? params_.ddcu_mbsu[static_cast<std::size_t>(d)] : -1;
}

bool EpsSystem::ddcu_admits(int d) const
{
  const auto & unit = ddcus_[static_cast<std::size_t>(d)];
  if (unit.fault() || unit.state().over_temperature) {
    return false;
  }
  const int m = mbsu_of_ddcu(d);
  if (m < 0) {
    return false;
  }
  // Before the first step there is no bus state yet; accept on wiring alone.
  return !stepped_ || !state_.mbsus[static_cast<std::size_t>(m)].unstable;
}

LoadResult EpsSystem::request_load(const std::string & id, double power_w, int priority,
                                   int preferred_ddcu)
{
  LoadResult result{false, "", -1, 0.0, 0.0};
  if (id.empty()) {
    result.message = "load_id must not be empty";
    return result;
  }
  if (!std::isfinite(power_w) || power_w <= 0.0) {
    result.message = "power_w must be a positive finite value";
    return result;
  }
  if (priority < 0) {
    result.message = "priority must be >= 0";
    return result;
  }
  if (preferred_ddcu < -1 || preferred_ddcu >= n_ddcu()) {
    result.message = "ddcu_index " + std::to_string(preferred_ddcu) + " does not exist";
    return result;
  }

  const double requested = power_w * registry_.demand_scale();
  const double rated = params_.ddcu.rated_power_w;
  const double eta = std::clamp(params_.ddcu.efficiency, 1.0e-6, 1.0);
  const auto * existing_spec = registry_.find(id);
  int chosen = -1;
  double chosen_headroom = -1.0;
  bool fits_rating = false;
  double best_margin = 0.0;
  const int first = preferred_ddcu >= 0 ? preferred_ddcu : 0;
  const int last = preferred_ddcu >= 0 ? preferred_ddcu : n_ddcu() - 1;
  for (int d = first; d <= last; ++d) {
    if (!ddcu_admits(d)) {
      continue;
    }
    const double headroom = rated - registry_.requested_on_ddcu(d, id);
    if (headroom + kPowerEpsilonW < requested) {
      continue;
    }
    fits_rating = true;
    if (stepped_) {
      // Supply margin of the MBSU feeding this DDCU from the last step,
      // crediting back the load being updated if it already sits there.
      const auto & m = state_.mbsus[static_cast<std::size_t>(mbsu_of_ddcu(d))];
      double credit = 0.0;
      if (existing_spec != nullptr && mbsu_of_ddcu(existing_spec->ddcu) == mbsu_of_ddcu(d)) {
        credit = registry_.requested_w(*existing_spec) / eta;
      }
      const double margin = m.capability_w - m.supplied_w + credit;
      best_margin = std::max(best_margin, margin);
      if (requested / eta > margin + kPowerEpsilonW) {
        continue;
      }
    }
    if (headroom > chosen_headroom) {
      chosen = d;
      chosen_headroom = headroom;
    }
  }
  if (chosen < 0) {
    const std::string watts = std::to_string(static_cast<int>(requested)) + " W";
    if (fits_rating) {
      result.message = "insufficient supply margin for " + watts + ": " +
        std::to_string(static_cast<int>(std::max(best_margin, 0.0) * eta)) + " W available";
    } else if (preferred_ddcu >= 0) {
      result.message = "DDCU " + std::to_string(preferred_ddcu) + " cannot take " + watts;
    } else {
      result.message = "no available DDCU has " + watts + " of headroom";
    }
    return result;
  }

  registry_.upsert(loads::LoadSpec{id, power_w, priority, chosen});
  result.accepted = true;
  result.ddcu = chosen;
  result.granted_w = requested;
  result.supply_voltage_v = params_.ddcu.output_voltage_v -
    params_.ddcu.regulation_droop_v *
    std::min(registry_.requested_on_ddcu(chosen) / std::max(rated, 1.0), 1.0);
  result.message = "load '" + id + "' on DDCU " + std::to_string(chosen);
  return result;
}

LoadResult EpsSystem::release_load(const std::string & id)
{
  LoadResult result{false, "", -1, 0.0, 0.0};
  const auto * spec = registry_.find(id);
  if (spec == nullptr) {
    result.message = "unknown load '" + id + "'";
    return result;
  }
  result.ddcu = spec->ddcu;
  registry_.remove(id);
  result.accepted = true;
  result.message = "load '" + id + "' released";
  return result;
}

const EpsState & EpsSystem::step(double dt, const EpsInput & input)
{
  dt = std::max(dt, 0.0);
  state_.array = array_.step(dt, input.solar_flux_w_m2, input.in_eclipse);
  state_.ssu = ssu_.step(dt, state_.array.array_power_w);

  const std::size_t n_ch = bcdus_.size();
  const std::size_t n_m = mbsus_.size();
  const std::size_t n_d = ddcus_.size();
  const double eta = std::clamp(params_.ddcu.efficiency, 1.0e-6, 1.0);

  // 1. What each channel can deliver to its MBSU this step.
  std::vector<bcdu::BcduInput> channel_in(n_ch);
  std::vector<double> capability(n_ch, 0.0);
  std::vector<double> channel_v(n_ch, 0.0);
  state_.supply_capability_w = 0.0;
  for (std::size_t c = 0; c < n_ch; ++c) {
    channel_in[c].ssu_voltage_v = state_.ssu.bus_voltage_v;
    channel_in[c].source_power_w = state_.ssu.available_power_w / static_cast<double>(n_ch);
    channel_in[c].load_power_w = 0.0;
    capability[c] = bcdus_[c].supply_capability_w(dt, channel_in[c], orus_by_channel_[c]);
    channel_v[c] = bcdus_[c].expected_bus_voltage_v(dt, channel_in[c], orus_by_channel_[c]);
    if (params_.channel_mbsu[c] >= 0 && !channel_offline_[c]) {
      state_.supply_capability_w += capability[c];
    }
  }

  std::vector<std::vector<mbsu::MbsuSource>> sources(n_m);
  for (std::size_t c = 0; c < n_ch; ++c) {
    const int m = params_.channel_mbsu[c];
    if (m >= 0) {
      sources[static_cast<std::size_t>(m)].push_back(
        mbsu::MbsuSource{static_cast<int>(c), !channel_offline_[c], channel_v[c],
          capability[c]});
    }
  }

  // 2. MBSU availability before any demand is routed.
  std::vector<double> mbsu_capability(n_m, 0.0);
  std::vector<double> mbsu_voltage(n_m, 0.0);
  std::vector<bool> mbsu_ok(n_m, false);
  for (std::size_t m = 0; m < n_m; ++m) {
    const auto & s = mbsus_[m].route(sources[m], 0.0);
    mbsu_capability[m] = s.capability_w;
    mbsu_voltage[m] = s.bus_voltage_v;
    mbsu_ok[m] = !s.unstable;
  }

  // 3. Load demand per DDCU, shedding down to the rating.
  const auto & specs = registry_.loads();
  state_.loads.assign(specs.size(), loads::LoadState{});
  std::vector<double> ddcu_requested(n_d, 0.0);
  std::vector<double> ddcu_kept(n_d, 0.0);
  std::vector<bool> ddcu_ok(n_d, false);
  std::vector<std::vector<std::size_t>> on_ddcu(n_d);
  for (std::size_t i = 0; i < specs.size(); ++i) {
    auto & ls = state_.loads[i];
    ls.id = specs[i].id;
    ls.priority = specs[i].priority;
    ls.ddcu = specs[i].ddcu;
    ls.requested_w = registry_.requested_w(specs[i]);
    ls.served_w = 0.0;
    ls.shed = false;
    if (ls.ddcu >= 0 && static_cast<std::size_t>(ls.ddcu) < n_d) {
      on_ddcu[static_cast<std::size_t>(ls.ddcu)].push_back(i);
      ddcu_requested[static_cast<std::size_t>(ls.ddcu)] += ls.requested_w;
    } else {
      ls.shed = true;
    }
  }
  for (std::size_t d = 0; d < n_d; ++d) {
    const int m = params_.ddcu_mbsu[d];
    ddcu_ok[d] = m >= 0 && mbsu_ok[static_cast<std::size_t>(m)] &&
      ddcus_[d].available(mbsu_voltage[static_cast<std::size_t>(m)]);
    if (!ddcu_ok[d]) {
      for (auto i : on_ddcu[d]) {
        state_.loads[i].shed = true;
      }
      continue;
    }
    ddcu_kept[d] = ddcu_requested[d];
    for (auto i : registry_.shed_order(on_ddcu[d])) {
      if (ddcu_kept[d] <= params_.ddcu.rated_power_w + kPowerEpsilonW) {
        break;
      }
      state_.loads[i].shed = true;
      ddcu_kept[d] -= state_.loads[i].requested_w;
    }
  }

  // 4. Shed further where an MBSU's sources cannot cover its DDCUs, then
  //    route what remains onto the channels.
  std::vector<double> ddcu_input(n_d, 0.0);
  std::vector<double> channel_load(n_ch, 0.0);
  std::vector<double> mbsu_routed(n_m, 0.0);
  for (std::size_t m = 0; m < n_m; ++m) {
    std::vector<std::size_t> candidates;
    double need = 0.0;
    for (std::size_t d = 0; d < n_d; ++d) {
      if (params_.ddcu_mbsu[d] != static_cast<int>(m) || !ddcu_ok[d]) {
        continue;
      }
      need += ddcus_[d].input_power_for(ddcu_kept[d]);
      for (auto i : on_ddcu[d]) {
        if (!state_.loads[i].shed) {
          candidates.push_back(i);
        }
      }
    }
    for (auto i : registry_.shed_order(candidates)) {
      if (need <= mbsu_capability[m] + kPowerEpsilonW) {
        break;
      }
      auto & ls = state_.loads[i];
      ls.shed = true;
      ddcu_kept[static_cast<std::size_t>(ls.ddcu)] -= ls.requested_w;
      need -= ls.requested_w / eta;
    }
    for (std::size_t d = 0; d < n_d; ++d) {
      if (params_.ddcu_mbsu[d] == static_cast<int>(m) && ddcu_ok[d]) {
        ddcu_kept[d] = std::max(ddcu_kept[d], 0.0);
        ddcu_input[d] = ddcus_[d].input_power_for(ddcu_kept[d]);
      }
    }
    const auto & routed = mbsus_[m].route(sources[m], std::max(need, 0.0));
    state_.mbsus[m] = routed;
    for (std::size_t k = 0; k < sources[m].size(); ++k) {
      channel_load[static_cast<std::size_t>(sources[m][k].channel)] += routed.allocation_w[k];
    }
    mbsu_routed[m] = routed.supplied_w;
  }

  // 5. Channels carry their routed load, charging or discharging as needed.
  for (std::size_t c = 0; c < n_ch; ++c) {
    channel_in[c].load_power_w = channel_load[c];
    state_.channels[c] = bcdus_[c].step(dt, channel_in[c], orus_by_channel_[c]);
  }
  for (std::size_t i = 0; i < orus_.size(); ++i) {
    if (!stepped_) {
      orus_[i]->settle_temperature();
    }
    state_.orus[i] = orus_[i]->state();
  }

  // 6. A channel that delivered less than routed browns out its MBSU's DDCUs.
  std::vector<double> mbsu_delivered(n_m, 0.0);
  for (std::size_t c = 0; c < n_ch; ++c) {
    const int m = params_.channel_mbsu[c];
    if (m >= 0) {
      mbsu_delivered[static_cast<std::size_t>(m)] += state_.channels[c].load_served_w;
    }
  }
  for (std::size_t d = 0; d < n_d; ++d) {
    const int m = params_.ddcu_mbsu[d];
    double ratio = 0.0;
    double voltage = 0.0;
    if (m >= 0) {
      const auto mi = static_cast<std::size_t>(m);
      ratio = mbsu_routed[mi] > kPowerEpsilonW ?
        std::min(mbsu_delivered[mi] / mbsu_routed[mi], 1.0) : 0.0;
      voltage = mbsu_ok[mi] ? mbsu_voltage[mi] : 0.0;
    }
    ddcu::DdcuInput din{};
    din.input_voltage_v = voltage;
    din.requested_w = ddcu_requested[d];
    din.demand_w = ddcu_ok[d] ? ddcu_kept[d] : 0.0;
    din.input_limit_w = ddcu_input[d] * ratio;
    ddcus_[d].step(dt, din);
    if (!stepped_) {
      ddcus_[d].settle_temperature();
    }
    state_.ddcus[d] = ddcus_[d].state();

    const double fraction = ddcu_kept[d] > kPowerEpsilonW ?
      std::min(state_.ddcus[d].output_power_w / ddcu_kept[d], 1.0) : 0.0;
    for (auto i : on_ddcu[d]) {
      auto & ls = state_.loads[i];
      ls.served_w = ls.shed ? 0.0 : ls.requested_w * fraction;
    }
  }

  stepped_ = true;
  summarise();
  return state_;
}

void EpsSystem::summarise()
{
  state_.generation_w = state_.array.array_power_w;
  state_.available_power_w = state_.ssu.available_power_w;

  state_.primary_load_w = 0.0;
  state_.battery_power_w = 0.0;
  state_.shunted_power_w = 0.0;
  state_.conversion_loss_w = 0.0;
  state_.bcdu_fault = false;
  for (const auto & ch : state_.channels) {
    state_.primary_load_w += ch.load_served_w;
    state_.battery_power_w += ch.battery_power_w;
    state_.shunted_power_w += ch.shunted_power_w;
    state_.conversion_loss_w += ch.conversion_loss_w;
    state_.bcdu_fault = state_.bcdu_fault || ch.faulted;
  }
  state_.power_balance_w = state_.available_power_w - state_.primary_load_w;

  state_.load_demand_w = 0.0;
  state_.load_served_w = 0.0;
  state_.shed_loads = 0;
  for (const auto & ls : state_.loads) {
    state_.load_demand_w += ls.requested_w;
    state_.load_served_w += ls.served_w;
    if (ls.shed) {
      ++state_.shed_loads;
    }
  }
  state_.unserved_load_w = std::max(state_.load_demand_w - state_.load_served_w, 0.0);

  state_.distribution_loss_w = 0.0;
  state_.ddcu_fault = false;
  state_.ddcu_overload = false;
  for (std::size_t i = 0; i < state_.ddcus.size(); ++i) {
    const auto & d = state_.ddcus[i];
    state_.distribution_loss_w += d.heat_w;
    const bool has_load = d.requested_w > kPowerEpsilonW;
    // A DDCU left without primary power by an unstable MBSU bus is the MBSU's
    // fault, not the DDCU's.
    const int m = params_.ddcu_mbsu[i];
    const bool bus_down = m >= 0 && state_.mbsus[static_cast<std::size_t>(m)].unstable;
    state_.ddcu_fault = state_.ddcu_fault || (has_load &&
      (d.mode == ddcu::DdcuMode::TRIPPED || (d.mode == ddcu::DdcuMode::OFF && !bus_down)));
    state_.ddcu_overload = state_.ddcu_overload || d.mode == ddcu::DdcuMode::OVERLOAD;
  }
  state_.mbsu_unstable = false;
  state_.redundancy_lost = false;
  for (const auto & m : state_.mbsus) {
    state_.mbsu_unstable = state_.mbsu_unstable || m.unstable;
    state_.redundancy_lost = state_.redundancy_lost || (!m.unstable && m.redundancy_lost);
  }
  state_.feeder_open = false;
  for (std::size_t c = 0; c < channel_offline_.size(); ++c) {
    state_.feeder_open = state_.feeder_open ||
      (channel_offline_[c] && params_.channel_mbsu[c] >= 0);
  }

  state_.battery_charge_w = 0.0;
  state_.battery_discharge_w = 0.0;
  state_.stored_energy_wh = 0.0;
  state_.oru_failed = false;
  state_.oru_overheat = false;
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
    state_.oru_overheat = state_.oru_overheat || s.health == battery::BatteryHealth::OVERHEAT;
    const double capacity = orus_[i]->usable_capacity_ah();
    weighted_soc += s.soc * capacity;
    total_capacity += capacity;
  }
  state_.battery_soc = total_capacity > 0.0 ? weighted_soc / total_capacity : 0.0;

  // Before the first step nothing has been delivered or routed yet.
  const double tolerance = params_.bcdu.unserved_tolerance_w;
  state_.low_soc = state_.battery_soc < params_.low_soc_threshold;
  state_.load_shed = stepped_ && state_.shed_loads > 0;
  state_.array_degraded = params_.solar.array.degradation_factor < 1.0 - 1.0e-9;
  state_.battery_degraded = params_.battery.state_of_health < params_.min_state_of_health;
  state_.load_unserved = stepped_ && state_.unserved_load_w > tolerance;
  if (!stepped_) {
    state_.mbsu_unstable = false;
    state_.redundancy_lost = false;
  }
  state_.healthy = !state_.low_soc && !state_.load_unserved && !state_.load_shed &&
    !state_.bcdu_fault && !state_.oru_failed && !state_.oru_overheat &&
    !state_.ddcu_fault && !state_.ddcu_overload && !state_.mbsu_unstable &&
    !state_.redundancy_lost && !state_.feeder_open && !state_.array_degraded &&
    !state_.battery_degraded && !state_.array.gimbal_locked;

  if (state_.load_shed) {
    state_.status_message = std::to_string(state_.shed_loads) + " load(s) shed";
  } else if (state_.load_unserved) {
    state_.status_message = "station load not fully supplied";
  } else if (state_.mbsu_unstable) {
    state_.status_message = "MBSU bus unstable";
  } else if (state_.ddcu_fault) {
    state_.status_message = "DDCU fault";
  } else if (state_.bcdu_fault) {
    state_.status_message = "BCDU fault";
  } else if (state_.ddcu_overload) {
    state_.status_message = "DDCU overload";
  } else if (state_.low_soc) {
    state_.status_message = "battery state of charge below threshold";
  } else if (state_.oru_overheat) {
    state_.status_message = "battery ORU over temperature";
  } else if (state_.oru_failed) {
    state_.status_message = "battery ORU failed";
  } else if (state_.redundancy_lost) {
    state_.status_message = "MBSU source redundancy lost";
  } else if (state_.feeder_open) {
    state_.status_message = "MBSU channel feeder open";
  } else if (state_.battery_degraded) {
    state_.status_message = "battery state of health degraded";
  } else if (state_.array_degraded) {
    state_.status_message = "solar array output degraded";
  } else if (state_.array.gimbal_locked) {
    state_.status_message = "solar array gimbal locked";
  } else {
    state_.status_message = "nominal";
  }
}

}  // namespace system
}  // namespace ssos_eps
