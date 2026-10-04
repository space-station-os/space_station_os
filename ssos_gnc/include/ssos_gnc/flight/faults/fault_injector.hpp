#ifndef SSOS_GNC__FLIGHT__FAULTS__FAULT_INJECTOR_HPP_
#define SSOS_GNC__FLIGHT__FAULTS__FAULT_INJECTOR_HPP_

#include <functional>
#include <vector>

#include "ssos_gnc/flight/faults/fault_types.hpp"

namespace ssos_gnc
{

namespace flight
{

namespace faults
{

struct HealthState
{
  bool cmg_healthy{true};
  bool thruster_healthy{true};
  bool imu_healthy{true};
  bool star_tracker_healthy{true};

  double cmg_capability{1.0};

  bool all_healthy() const
  {
    return cmg_healthy && thruster_healthy && imu_healthy && star_tracker_healthy;
  }
};

struct FaultTransition
{
  FaultDefinition fault;
  bool activated{true};
};

class FaultInjector
{
public:
  FaultInjector();

  void add_fault(const FaultDefinition & fault);

  void inject_now(const FaultDefinition & fault);

  void reset();

  std::vector<FaultTransition> update(double dt);

  const HealthState & health() const {return health_;}
  const std::vector<FaultDefinition> & active_faults() const {return active_;}
  double scenario_time() const {return time_s_;}

  bool is_active(FaultType type) const;

private:
  void recompute_health();

  std::vector<FaultDefinition> scheduled_;
  std::vector<FaultDefinition> active_;
  HealthState health_;
  double time_s_{0.0};
};
}  // namespace faults
}  // namespace flight
}  // namespace ssos_gnc

#endif  // SSOS_GNC__FLIGHT__FAULTS__FAULT_INJECTOR_HPP_
