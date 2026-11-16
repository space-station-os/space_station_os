#ifndef SSOS_GNC__FLIGHT__CONTROL__CONTROL_PARAMETERS_HPP_
#define SSOS_GNC__FLIGHT__CONTROL__CONTROL_PARAMETERS_HPP_

namespace ssos_gnc
{

namespace flight
{

struct PdGains
{
  double kp{300000.0};
  double kd{300000.0};
};

struct FilterParams
{
  double angle_alpha{0.0};
  double rate_alpha{0.0};

  static double alpha_from_tau(double tau_s, double dt_s);
};

struct DeadZoneParams
{
  double angle_on_deg{0.150};
  double angle_off_deg{0.100};
  double rate_on_dps{0.250};
  double rate_off_dps{0.150};
  bool enabled{true};
};

struct UnloadParams
{
  double gain{10.0};
  double completion_threshold{0.01};
};

struct ModeParams
{
  double min_dwell_s{1.0};
};

struct GncParameters
{
  PdGains cmg{300000.0, 300000.0};
  PdGains thruster{1000000.0, 1000000.0};
  FilterParams filter{};
  DeadZoneParams dead_zone{};
  UnloadParams unload{};
  ModeParams mode{};

  double gimbal_rate_limit{0.5};
};

GncParameters default_gnc_parameters();
}  // namespace flight
}  // namespace ssos_gnc

#endif  // SSOS_GNC__FLIGHT__CONTROL__CONTROL_PARAMETERS_HPP_
