#include "ssos_gnc/flight/faults/fault_injector.hpp"

#include <algorithm>

namespace ssos_gnc
{

namespace flight
{

namespace faults
{

FaultInjector::FaultInjector() = default;

void FaultInjector::add_fault(const FaultDefinition & fault)
{
  scheduled_.push_back(fault);
}  // namespace faults

void FaultInjector::inject_now(const FaultDefinition & fault)
{
  FaultDefinition f = fault;
  f.start_time_s = time_s_;
  active_.push_back(f);
  recompute_health();
}  // namespace flight

void FaultInjector::reset()
{
  scheduled_.clear();
  active_.clear();
  time_s_ = 0.0;
  health_ = HealthState{};
}  // namespace ssos_gnc

bool FaultInjector::is_active(FaultType type) const
{
  return std::any_of(
    active_.begin(), active_.end(),
    [type](const FaultDefinition & f) {return f.type == type;});
}

std::vector<FaultTransition> FaultInjector::update(double dt)
{
  std::vector<FaultTransition> transitions;

  if (dt > 0.0) {
    time_s_ += dt;
  }

  for (auto it = scheduled_.begin(); it != scheduled_.end(); ) {
    if (time_s_ >= it->start_time_s) {
      active_.push_back(*it);
      transitions.push_back(FaultTransition{*it, true});
      it = scheduled_.erase(it);
    } else {
      ++it;
    }
  }

  for (auto it = active_.begin(); it != active_.end(); ) {
    const bool expires = it->duration_s >= 0.0;
    if (expires && time_s_ >= (it->start_time_s + it->duration_s)) {
      transitions.push_back(FaultTransition{*it, false});
      it = active_.erase(it);
    } else {
      ++it;
    }
  }

  if (!transitions.empty()) {
    recompute_health();
  }
  return transitions;
}

void FaultInjector::recompute_health()
{
  health_ = HealthState{};

  for (const auto & f : active_) {
    switch (f.type) {
      case FaultType::CMG_FAILED:
        health_.cmg_healthy = false;
        health_.cmg_capability = 0.0;
        break;

      case FaultType::CMG_DEGRADED:
        health_.cmg_capability = std::min(
          health_.cmg_capability, std::max(0.0, std::min(1.0, f.magnitude)));
        break;

      case FaultType::THRUSTER_STUCK_OPEN:
      case FaultType::THRUSTER_STUCK_CLOSED:
        health_.thruster_healthy = false;
        break;

      case FaultType::IMU_BIAS:
      case FaultType::IMU_DRIFT:
      case FaultType::IMU_NOISE:
      case FaultType::IMU_STUCK:
        health_.imu_healthy = false;
        break;

      case FaultType::STAR_TRACKER_DROPOUT:
        health_.star_tracker_healthy = false;
        break;

      default:
        break;
    }
  }
}
}
}
}
