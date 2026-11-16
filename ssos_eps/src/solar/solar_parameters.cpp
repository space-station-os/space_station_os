#include "ssos_eps/solar/solar_parameters.hpp"

namespace ssos_eps
{
namespace solar
{

SolarArrayParams default_solar_array_params()
{
  SolarArrayParams p{};
  // 375 m^2 at 16.5% gives ~84 kW at 1361 W/m^2, the ISS-class peak used by
  // the legacy space_station_eps solar controller.
  p.area_m2 = 375.0;
  p.cell_efficiency = 0.165;
  p.degradation_factor = 1.0;
  p.sunrise_ramp_s = 480.0;
  p.derate_with_beta_cosine = true;
  p.beta_max_deg = 75.0;
  p.beta_precession_orbits = 10;
  p.beta_lock_limit_deg = 72.0;
  p.gimbal_lock_derate = 0.6;
  p.use_fixed_beta = false;
  p.fixed_beta_deg = 0.0;
  return p;
}

PanelThermalParams default_panel_thermal_params()
{
  PanelThermalParams p{};
  p.ambient_temp_c = 20.0;
  p.temp_rise_per_kw_c = 0.4;
  p.time_constant_s = 300.0;
  return p;
}

SsuParams default_ssu_params()
{
  SsuParams p{};
  p.voltage_setpoint_v = 160.0;
  p.voltage_ripple_v = 0.6;
  p.ripple_period_s = 1350.0;
  p.efficiency = 0.985;
  p.min_input_power_w = 1.0;
  return p;
}

SolarParameters default_solar_parameters()
{
  SolarParameters p{};
  p.array = default_solar_array_params();
  p.thermal = default_panel_thermal_params();
  p.ssu = default_ssu_params();
  return p;
}

}  // namespace solar
}  // namespace ssos_eps
