#ifndef SSOS_EPS__BATTERY__BATTERY_ORU_MODEL_HPP_
#define SSOS_EPS__BATTERY__BATTERY_ORU_MODEL_HPP_

#include "ssos_eps/battery/battery_parameters.hpp"

// Equivalent-circuit Li-ion battery ORU: open-circuit voltage as a function of
// state of charge in series with an internal resistance, coulomb counting for
// SOC, and a lumped thermal node heated by I^2 R and cooled by the cold plate.
//
// Sign convention follows BCDUStatus: positive current and power flow OUT of
// the battery (discharge), negative flow IN (charge). No ROS.

namespace ssos_eps
{
namespace battery
{

enum class BatteryMode
{
  IDLE,
  CHARGING,
  DISCHARGING
};

enum class BatteryHealth
{
  GOOD,
  OVERHEAT,
  FAILED
};

struct BatteryOruState
{
  double soc;                  // state of charge [0, 1]
  double ocv_v;                // pack open-circuit voltage [V]
  double terminal_voltage_v;   // pack terminal voltage under load [V]
  double current_a;            // +discharge / -charge [A]
  double power_w;              // terminal power, +discharge / -charge [W]
  double heat_w;               // resistive heat generation [W]
  double temperature_c;        // lumped pack temperature [C]
  double stored_energy_wh;     // energy recoverable at open circuit [Wh]
  BatteryMode mode;
  BatteryHealth health;
};

class BatteryOruModel
{
public:
  explicit BatteryOruModel(const BatteryOruParams & params = default_battery_oru_params());

  void reset();
  void reset(double soc, double temperature_c);

  /// Pack open-circuit voltage at a given SOC [V].
  double open_circuit_voltage(double soc) const;

  /// Energy recoverable at open circuit between SOC 0 and soc [Wh].
  double stored_energy_wh(double soc) const;

  /// Largest discharge power the pack can deliver over dt [W] (>= 0).
  double max_discharge_power_w(double dt) const;

  /// Largest charge power the pack can absorb over dt [W] (>= 0).
  double max_charge_power_w(double dt) const;

  /// Apply a terminal power request for dt. Positive discharges, negative
  /// charges. The request is clipped to the current, voltage and SOC limits.
  /// @return the terminal power actually exchanged [W]
  double step(double dt, double power_request_w);

  /// A failed ORU is isolated: it carries no current until cleared.
  void set_failed(bool failed);
  bool failed() const {return failed_;}

  void set_params(const BatteryOruParams & params) {params_ = params;}
  const BatteryOruParams & params() const {return params_;}
  const BatteryOruState & state() const {return state_;}

  /// Usable capacity after state-of-health derating [Ah].
  double usable_capacity_ah() const;

private:
  double discharge_current_limit_a(double dt) const;
  double charge_current_limit_a(double dt) const;
  void update_derived(double current_a, double dt);

  BatteryOruParams params_;
  BatteryOruState state_{};
  bool failed_{false};
};

}  // namespace battery
}  // namespace ssos_eps

#endif  // SSOS_EPS__BATTERY__BATTERY_ORU_MODEL_HPP_
