#ifndef SSOS_EPS__BCDU__BCDU_PARAMETERS_HPP_
#define SSOS_EPS__BCDU__BCDU_PARAMETERS_HPP_

// Parameters for one battery charge/discharge unit (BCDU), i.e. one power
// channel's interface between the primary bus and its battery ORUs. Voltages
// and currents are on the bus side.

namespace ssos_eps
{
namespace bcdu
{

struct BcduParams
{
  double charge_enter_v;          // SSU voltage at or above which the source is available [V]
  double discharge_enter_v;       // SSU voltage at or below which the source is lost [V]
  double regulation_voltage_v;    // bus voltage held while discharging [V]
  double regulation_min_v;        // lowest permitted regulation setpoint [V]
  double regulation_max_v;        // highest permitted regulation setpoint [V]
  double max_charge_current_a;    // bus-side charge current limit [A]
  double max_discharge_current_a; // bus-side discharge current limit [A]
  double charge_efficiency;       // bus -> battery conversion efficiency [-]
  double discharge_efficiency;    // battery -> bus conversion efficiency [-]
  double deadband_w;              // power imbalance ignored before switching mode [W]
  double unserved_tolerance_w;    // unmet load tolerated before declaring degraded [W]
};

BcduParams default_bcdu_params();

}  // namespace bcdu
}  // namespace ssos_eps

#endif  // SSOS_EPS__BCDU__BCDU_PARAMETERS_HPP_
