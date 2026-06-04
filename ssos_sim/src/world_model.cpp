#include "ssos_sim/world_model.hpp"
#include <cmath>

namespace ssos_sim
{

/// Default LEO world model.
/// Models basic orbital eclipse cycling and holds atmosphere/thermal state.
/// Physics refinement (drag, J2, thermal dynamics) comes in later iterations.
class DefaultWorldModel : public WorldModel
{
public:
  DefaultWorldModel() = default;

  void initialize(const WorldState & initial_state) override
  {
    state_ = initial_state;
    orbital_period_s_ = 2.0 * M_PI * (6371.0 + state_.altitude_km) * 1000.0 / state_.orbital_velocity_mps;
    eclipse_fraction_ = 0.35;  // ~35% of orbit in shadow for ISS-like altitude
  }

  void step(double dt) override
  {
    elapsed_s_ += dt;

    // Eclipse cycling based on orbital period
    double orbit_phase = std::fmod(elapsed_s_, orbital_period_s_) / orbital_period_s_;
    bool was_in_eclipse = state_.in_eclipse;
    state_.in_eclipse = orbit_phase > (1.0 - eclipse_fraction_);

    // Solar flux: full sun or zero in eclipse
    state_.solar_flux_w_m2 = state_.in_eclipse ? 0.0 : 1361.0;

    // Slight cabin temp drift during eclipse (simplified thermal model)
    if (state_.in_eclipse) {
      state_.cabin_temp_celsius -= 0.001 * dt;  // Very slow cooling
    } else {
      // Drift back toward nominal 22°C
      state_.cabin_temp_celsius += 0.001 * dt * (22.0 - state_.cabin_temp_celsius);
    }

    // Clamp cabin temp to reasonable range
    state_.cabin_temp_celsius = std::max(18.0, std::min(26.0, state_.cabin_temp_celsius));

    (void)was_in_eclipse;  // Available for event detection in future
  }

  WorldState get_state() const override
  {
    return state_;
  }

  void apply_perturbation(const std::string & param, double value) override
  {
    if (param == "atmospheric_o2_pct") {
      state_.atmospheric_o2_pct = value;
    } else if (param == "atmospheric_co2_ppm") {
      state_.atmospheric_co2_ppm = value;
    } else if (param == "cabin_pressure_kpa") {
      state_.cabin_pressure_kpa = value;
    } else if (param == "cabin_temp_celsius") {
      state_.cabin_temp_celsius = value;
    }
  }

private:
  WorldState state_;
  double elapsed_s_{0.0};
  double orbital_period_s_{5520.0};  // ~92 minutes default
  double eclipse_fraction_{0.35};
};

// Factory function
std::unique_ptr<WorldModel> create_default_world_model()
{
  return std::make_unique<DefaultWorldModel>();
}

}  // namespace ssos_sim