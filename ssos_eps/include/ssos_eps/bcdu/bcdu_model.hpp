#ifndef SSOS_EPS__BCDU__BCDU_MODEL_HPP_
#define SSOS_EPS__BCDU__BCDU_MODEL_HPP_

#include <string>
#include <vector>

#include "ssos_eps/battery/battery_oru_model.hpp"
#include "ssos_eps/bcdu/bcdu_parameters.hpp"

// Battery charge/discharge unit for one power channel.
//
// Source availability follows the SSU voltage with hysteresis. With the source
// available, surplus array power charges the batteries (within the bus-side
// charge limit and what the ORUs can accept) and any remainder is shunted; a
// deficit is made up by discharging. Without the source the batteries carry
// the whole channel load at the regulation voltage. Power is shared between
// ORUs in proportion to what each can deliver or absorb. No ROS.

namespace ssos_eps
{
namespace bcdu
{

enum class BcduMode
{
  IDLE,
  CHARGING,
  DISCHARGING,
  FAULT
};

/// Lower-case mode name as carried in BCDUStatus.mode.
std::string to_string(BcduMode mode);

struct BcduInput
{
  double ssu_voltage_v;        // regulated array voltage [V]
  double source_power_w;       // array power available to this channel [W]
  double load_power_w;         // channel load demand [W]
};

struct BcduState
{
  BcduMode mode;
  bool source_available;       // hysteresis output on the SSU voltage
  bool faulted;                // BCDU isolated by a fault
  bool degraded;               // load unmet or an ORU failed
  double bus_voltage_v;        // channel bus voltage [V]
  double regulation_voltage_v; // discharge regulation setpoint [V]
  double current_draw_a;       // bus side, +discharge to bus / -charging [A]
  double bus_power_w;          // bus side, +discharge to bus / -charging [W]
  double battery_power_w;      // battery terminals, +discharge / -charge [W]
  double load_served_w;        // load actually supplied [W]
  double unserved_load_w;      // load that could not be supplied [W]
  double shunted_power_w;      // array power left unused [W]
  double conversion_loss_w;    // BCDU conversion losses [W]
  int failed_orus;             // ORUs on this channel that are failed
  std::string status_message;
};

class BcduModel
{
public:
  explicit BcduModel(const BcduParams & params = default_bcdu_params());

  void reset();

  /// Advance the channel by dt, charging or discharging its ORUs.
  const BcduState & step(double dt, const BcduInput & input,
                         std::vector<battery::BatteryOruModel> & orus);

  /// A faulted BCDU isolates its batteries; the source still feeds the load.
  void set_fault(bool faulted) {faulted_ = faulted;}
  bool fault() const {return faulted_;}

  void set_params(const BcduParams & params) {params_ = params;}
  const BcduParams & params() const {return params_;}
  const BcduState & state() const {return state_;}

private:
  /// Share a terminal power request across the ORUs (+discharge, -charge).
  /// @return the terminal power actually exchanged [W]
  static double distribute(double dt, double power_w,
                           std::vector<battery::BatteryOruModel> & orus);

  BcduParams params_;
  BcduState state_{};
  bool faulted_{false};
};

}  // namespace bcdu
}  // namespace ssos_eps

#endif  // SSOS_EPS__BCDU__BCDU_MODEL_HPP_
