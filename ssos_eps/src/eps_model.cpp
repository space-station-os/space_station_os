#include "ssos_eps/eps_model.hpp"

#include <algorithm>

namespace ssos_eps
{

EpsModel::EpsModel(const EpsConfig & config)
: config_(config)
{
  state_.load_demand_w = config_.load_demand_w;
  state_.battery_soc = std::clamp(config_.initial_battery_soc, 0.0, 1.0);
  state_.healthy = state_.battery_soc > config_.low_soc_threshold;
}

EpsState EpsModel::update(const EpsInput & input)
{
  const double solar_flux_w_m2 = std::max(0.0, input.solar_flux_w_m2);
  const double dt_s = std::max(0.0, input.dt_s);

  if (input.in_eclipse) {
    state_.generation_w = 0.0;
  } else {
    state_.generation_w =
      solar_flux_w_m2 *
      config_.solar_array_area_m2 *
      config_.solar_array_efficiency;
  }

  state_.load_demand_w = config_.load_demand_w;
  state_.power_balance_w = state_.generation_w - state_.load_demand_w;

  if (config_.battery_capacity_wh > 0.0) {
    const double energy_delta_wh = state_.power_balance_w * dt_s / 3600.0;
    const double soc_delta = energy_delta_wh / config_.battery_capacity_wh;

    state_.battery_soc =
      std::clamp(state_.battery_soc + soc_delta, 0.0, 1.0);
  }

  state_.healthy = state_.battery_soc > config_.low_soc_threshold;

  return state_;
}

const EpsState & EpsModel::state() const
{
  return state_;
}

const EpsConfig & EpsModel::config() const
{
  return config_;
}

}  // namespace ssos_eps
