#include "ssos_eps/mbsu/mbsu_model.hpp"

#include <algorithm>

namespace ssos_eps
{
namespace mbsu
{

MbsuModel::MbsuModel(const MbsuParams & params)
: params_(params)
{
  reset();
}

void MbsuModel::reset()
{
  state_ = MbsuState{};
  state_.unstable = true;
  state_.status_message = "no source";
  faulted_ = false;
}

bool MbsuModel::usable(const MbsuSource & source) const
{
  return source.connected && source.capability_w > 0.0 &&
         source.voltage_v >= params_.min_source_voltage_v &&
         source.voltage_v <= params_.max_source_voltage_v;
}

const MbsuState & MbsuModel::route(const std::vector<MbsuSource> & sources, double demand_w)
{
  const std::size_t n = sources.size();
  state_.source_channels.assign(n, -1);
  state_.source_available.assign(n, false);
  state_.allocation_w.assign(n, 0.0);
  state_.available_sources = 0;
  state_.capability_w = 0.0;
  state_.demand_w = std::max(demand_w, 0.0);
  state_.faulted = faulted_;

  double voltage_sum = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    state_.source_channels[i] = sources[i].channel;
    if (!faulted_ && usable(sources[i])) {
      state_.source_available[i] = true;
      ++state_.available_sources;
      state_.capability_w += sources[i].capability_w;
      voltage_sum += sources[i].voltage_v;
    }
  }

  state_.bus_voltage_v = state_.available_sources > 0 ?
    voltage_sum / state_.available_sources : 0.0;
  state_.unstable = faulted_ || state_.available_sources < std::max(params_.min_sources, 1);
  state_.redundancy_lost = state_.available_sources < params_.redundant_sources;

  state_.supplied_w = state_.unstable ? 0.0 : std::min(state_.demand_w, state_.capability_w);
  state_.shortfall_w = state_.demand_w - state_.supplied_w;
  if (state_.capability_w > 0.0 && state_.supplied_w > 0.0) {
    for (std::size_t i = 0; i < n; ++i) {
      if (state_.source_available[i]) {
        state_.allocation_w[i] =
          state_.supplied_w * sources[i].capability_w / state_.capability_w;
      }
    }
  }

  if (faulted_) {
    state_.status_message = "MBSU fault: bus isolated";
  } else if (state_.unstable) {
    state_.status_message = "no usable source channel";
  } else if (state_.shortfall_w > 1.0e-6) {
    state_.status_message = "demand exceeds source capability";
  } else if (state_.redundancy_lost) {
    state_.status_message = "source redundancy lost";
  } else {
    state_.status_message = "nominal";
  }
  return state_;
}

}  // namespace mbsu
}  // namespace ssos_eps
