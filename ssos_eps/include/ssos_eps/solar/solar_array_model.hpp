#ifndef SSOS_EPS__SOLAR__SOLAR_ARRAY_MODEL_HPP_
#define SSOS_EPS__SOLAR__SOLAR_ARRAY_MODEL_HPP_

#include "ssos_eps/solar/solar_parameters.hpp"

// Photovoltaic array driven by the environment's solar flux and eclipse flag.
// Output = flux * area * efficiency * illumination * beta derate * degradation.
// Illumination ramps up with a half-cosine after each sunrise while the arrays
// re-acquire the sun, and drops to zero on eclipse entry. Beta precesses once
// every beta_precession_orbits sunrises; above the lock limit the SARJ is
// treated as gimbal-locked and an extra derate applies. No ROS.

namespace ssos_eps
{
namespace solar
{

struct SolarArrayState
{
  bool sunlit;                 // true outside eclipse
  double illumination;         // pointing/penumbra factor [0, 1]
  double beta_deg;             // solar beta angle [deg]
  double beta_derate;          // output factor from beta and gimbal lock [-]
  bool gimbal_locked;          // |beta| at or above the lock limit
  double incident_flux_w_m2;   // flux reaching the cells [W/m^2]
  double array_power_w;        // DC power at the array terminals [W]
  double panel_temperature_c;  // lumped panel temperature [C]
  int orbit_count;             // sunrises since reset
};

class SolarArrayModel
{
public:
  explicit SolarArrayModel(const SolarArrayParams & array = default_solar_array_params(),
                           const PanelThermalParams & thermal =
                           default_panel_thermal_params());

  void reset();

  /// Advance by dt.
  /// @param dt                timestep [s]
  /// @param solar_flux_w_m2   solar flux at the station [W/m^2]
  /// @param in_eclipse        true while the station is in Earth's shadow
  const SolarArrayState & step(double dt, double solar_flux_w_m2, bool in_eclipse);

  /// Beta angle for the current orbit count.
  double beta_angle_deg() const;

  void set_params(const SolarArrayParams & array, const PanelThermalParams & thermal);
  const SolarArrayParams & array_params() const {return array_;}
  const PanelThermalParams & thermal_params() const {return thermal_;}
  const SolarArrayState & state() const {return state_;}

private:
  SolarArrayParams array_;
  PanelThermalParams thermal_;
  SolarArrayState state_{};
  double time_since_sunrise_s_{0.0};
  bool initialized_{false};
};

}  // namespace solar
}  // namespace ssos_eps

#endif  // SSOS_EPS__SOLAR__SOLAR_ARRAY_MODEL_HPP_
