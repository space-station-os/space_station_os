#ifndef SSOS_EPS__SOLAR__SOLAR_PARAMETERS_HPP_
#define SSOS_EPS__SOLAR__SOLAR_PARAMETERS_HPP_

// Parameters for the photovoltaic arrays, the solar alpha rotary joint (SARJ)
// pointing model and the sequential shunt unit (SSU). SI units unless the
// field name says otherwise. Factory functions provide ISS-class defaults.

namespace ssos_eps
{
namespace solar
{

/// Photovoltaic array and pointing.
struct SolarArrayParams
{
  double area_m2;                 // total active cell area, all wings [m^2]
  double cell_efficiency;         // beginning-of-life conversion efficiency [-]
  double degradation_factor;      // fraction of BOL output remaining [-]
  double sunrise_ramp_s;          // array re-acquisition time after eclipse exit [s]
  bool derate_with_beta_cosine;   // apply cos(beta) to incident flux
  double beta_max_deg;            // peak solar beta angle over a precession cycle [deg]
  int beta_precession_orbits;     // orbits per full beta cycle [-]
  double beta_lock_limit_deg;     // |beta| above which the SARJ cannot track [deg]
  double gimbal_lock_derate;      // extra output factor while gimbal-locked [-]
  bool use_fixed_beta;            // hold beta at fixed_beta_deg instead of precessing
  double fixed_beta_deg;          // beta used when use_fixed_beta is set [deg]
};

/// Lumped first-order panel temperature.
struct PanelThermalParams
{
  double ambient_temp_c;          // panel temperature with no generation [C]
  double temp_rise_per_kw_c;      // steady-state rise per kW generated [C/kW]
  double time_constant_s;         // first-order thermal time constant [s]
};

/// Sequential shunt unit regulating array output onto the primary bus.
struct SsuParams
{
  double voltage_setpoint_v;      // regulated primary bus voltage [V]
  double voltage_ripple_v;        // peak regulation ripple [V]
  double ripple_period_s;         // ripple period [s]
  double efficiency;              // conversion efficiency [-]
  double min_input_power_w;       // array power below which the SSU drops out [W]
};

/// Complete solar generation parameter set.
struct SolarParameters
{
  SolarArrayParams array;
  PanelThermalParams thermal;
  SsuParams ssu;
};

SolarArrayParams default_solar_array_params();
PanelThermalParams default_panel_thermal_params();
SsuParams default_ssu_params();
SolarParameters default_solar_parameters();

}  // namespace solar
}  // namespace ssos_eps

#endif  // SSOS_EPS__SOLAR__SOLAR_PARAMETERS_HPP_
