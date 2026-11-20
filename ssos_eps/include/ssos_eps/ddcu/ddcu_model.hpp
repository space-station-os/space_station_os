#ifndef SSOS_EPS__DDCU__DDCU_MODEL_HPP_
#define SSOS_EPS__DDCU__DDCU_MODEL_HPP_

#include <algorithm>
#include <string>

#include "ssos_eps/ddcu/ddcu_parameters.hpp"

// DC-to-DC converter unit. Accepts primary power inside its input window and
// regulates it onto the secondary bus with a small load-dependent droop.
// Output is limited to the rating; losses (efficiency plus a standby term)
// heat a lumped thermal node that trips the unit when it overheats and
// clears once it has cooled. No ROS.

namespace ssos_eps
{
namespace ddcu
{

enum class DdcuMode
{
  OFF,          // input outside the window: no output
  REGULATING,
  OVERLOAD,     // requested load above the rating; output held at rating
  TRIPPED       // isolated by a fault or over-temperature
};

/// Lower-case mode name as carried in DDCUStatus.mode.
std::string to_string(DdcuMode mode);

struct DdcuInput
{
  double input_voltage_v;      // primary bus voltage at the DDCU [V]
  double requested_w;          // load requested before any shedding [W]
  double demand_w;             // load to supply after shedding [W]
  double input_limit_w;        // primary power the MBSU can deliver [W]
};

struct DdcuState
{
  DdcuMode mode;
  double input_voltage_v;      // [V]
  double output_voltage_v;     // [V]
  double requested_w;          // [W]
  double output_power_w;       // delivered to loads [W]
  double input_power_w;        // drawn from the primary bus [W]
  double heat_w;               // conversion losses [W]
  double temperature_c;        // [C]
  double load_fraction;        // output / rating [-]
  bool faulted;                // isolated by a commanded fault
  bool over_temperature;       // thermal trip latched
  std::string status_message;
};

class DdcuModel
{
public:
  explicit DdcuModel(const DdcuParams & params = default_ddcu_params());

  void reset();

  /// Primary power needed to deliver output_w [W].
  double input_power_for(double output_w) const;

  /// True if the DDCU can regulate from this primary voltage right now.
  bool available(double input_voltage_v) const;

  const DdcuState & step(double dt, const DdcuInput & input);

  void set_fault(bool faulted) {faulted_ = faulted;}
  bool fault() const {return faulted_;}

  /// Heat added to the thermal node on top of the conversion losses, e.g. a
  /// failed fan or a cold-plate blockage [W].
  void set_extra_heat_w(double heat_w) {extra_heat_w_ = std::max(heat_w, 0.0);}

  /// Jump to the thermal equilibrium of the present losses, without any
  /// extra heat.
  void settle_temperature();

  void set_params(const DdcuParams & params) {params_ = params;}
  const DdcuParams & params() const {return params_;}
  const DdcuState & state() const {return state_;}

private:
  DdcuParams params_;
  DdcuState state_{};
  bool faulted_{false};
  bool over_temperature_{false};
  double extra_heat_w_{0.0};
};

}  // namespace ddcu
}  // namespace ssos_eps

#endif  // SSOS_EPS__DDCU__DDCU_MODEL_HPP_
