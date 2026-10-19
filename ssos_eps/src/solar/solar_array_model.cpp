#include "ssos_eps/solar/solar_array_model.hpp"

#include <algorithm>
#include <cmath>

#include "ssos_eps/common/interpolation.hpp"
#include "ssos_eps/common/units.hpp"

namespace ssos_eps
{
namespace solar
{

SolarArrayModel::SolarArrayModel(const SolarArrayParams & array,
                                 const PanelThermalParams & thermal)
: array_(array), thermal_(thermal)
{
  reset();
}

void SolarArrayModel::reset()
{
  state_ = SolarArrayState{};
  state_.panel_temperature_c = thermal_.ambient_temp_c;
  state_.beta_derate = 1.0;
  time_since_sunrise_s_ = 0.0;
  initialized_ = false;
}

void SolarArrayModel::set_params(const SolarArrayParams & array,
                                 const PanelThermalParams & thermal)
{
  array_ = array;
  thermal_ = thermal;
}

double SolarArrayModel::beta_angle_deg() const
{
  if (array_.use_fixed_beta) {
    return array_.fixed_beta_deg;
  }
  if (array_.beta_precession_orbits <= 0) {
    return 0.0;
  }
  const double phase = 2.0 * units::PI * static_cast<double>(state_.orbit_count) /
    static_cast<double>(array_.beta_precession_orbits);
  return array_.beta_max_deg * std::sin(phase);
}

const SolarArrayState & SolarArrayModel::step(double dt, double solar_flux_w_m2,
                                              bool in_eclipse)
{
  dt = std::max(dt, 0.0);
  const bool sunlit = !in_eclipse;

  if (!initialized_) {
    // The station starts already tracking the sun if it begins in daylight.
    time_since_sunrise_s_ = sunlit ? array_.sunrise_ramp_s : 0.0;
    initialized_ = true;
  } else if (sunlit && !state_.sunlit) {
    ++state_.orbit_count;
    time_since_sunrise_s_ = 0.0;
  }
  state_.sunlit = sunlit;

  if (sunlit) {
    time_since_sunrise_s_ += dt;
    state_.illumination = array_.sunrise_ramp_s > 0.0 ?
      common::cosine_rise(time_since_sunrise_s_ / array_.sunrise_ramp_s) : 1.0;
  } else {
    state_.illumination = 0.0;
  }

  state_.beta_deg = beta_angle_deg();
  const double abs_beta = std::abs(state_.beta_deg);
  double derate = 1.0;
  if (array_.derate_with_beta_cosine) {
    derate = std::max(0.0, std::cos(units::deg_to_rad(std::min(abs_beta, 89.9))));
  }
  state_.gimbal_locked = abs_beta >= array_.beta_lock_limit_deg;
  if (state_.gimbal_locked) {
    derate *= array_.gimbal_lock_derate;
  }
  state_.beta_derate = derate;

  const double flux = sunlit ? std::max(solar_flux_w_m2, 0.0) : 0.0;
  state_.incident_flux_w_m2 = flux * state_.illumination * derate;
  state_.array_power_w = state_.incident_flux_w_m2 * array_.area_m2 *
    array_.cell_efficiency * std::clamp(array_.degradation_factor, 0.0, 1.0);

  // First-order lag toward the generation-dependent steady temperature. The
  // exact exponential keeps this stable for any dt.
  const double target_c = thermal_.ambient_temp_c +
    thermal_.temp_rise_per_kw_c * state_.array_power_w / 1000.0;
  const double alpha = thermal_.time_constant_s > 0.0 ?
    1.0 - std::exp(-dt / thermal_.time_constant_s) : 1.0;
  state_.panel_temperature_c += alpha * (target_c - state_.panel_temperature_c);

  return state_;
}

}  // namespace solar
}  // namespace ssos_eps
