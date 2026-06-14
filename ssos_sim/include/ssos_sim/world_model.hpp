#ifndef SSOS_SIM__WORLD_MODEL_HPP_
#define SSOS_SIM__WORLD_MODEL_HPP_

#include "space_station_interfaces/msg/world_state.hpp"

namespace ssos_sim
{

using WorldState = space_station_interfaces::msg::WorldState;

/// Abstract base class for world models.
/// Implementations provide the physics that drive the simulation environment.
class WorldModel
{
public:
  virtual ~WorldModel() = default;

  /// Set initial conditions
  virtual void initialize(const WorldState & initial_state) = 0;

  /// Advance the world model by dt seconds
  virtual void step(double dt) = 0;

  /// Get the current world state
  virtual WorldState get_state() const = 0;

  /// Apply an external perturbation (e.g. fault effect on environment)
  virtual void apply_perturbation(const std::string & param, double value)
  {
    (void)param;
    (void)value;
  }
};

}  // namespace ssos_sim

#endif  // SSOS_SIM__WORLD_MODEL_HPP_