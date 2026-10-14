#ifndef SSOS_EPS__SYSTEM__EPS_SYSTEM_HPP_
#define SSOS_EPS__SYSTEM__EPS_SYSTEM_HPP_

#include <string>
#include <vector>

#include "ssos_eps/battery/battery_oru_model.hpp"
#include "ssos_eps/bcdu/bcdu_model.hpp"
#include "ssos_eps/solar/solar_array_model.hpp"
#include "ssos_eps/solar/ssu_model.hpp"
#include "ssos_eps/system/eps_parameters.hpp"

// Top-level EPS generation and storage. Inputs the environment (solar flux,
// eclipse) and the station load; steps array -> SSU -> per-channel BCDU and
// batteries in a fixed order each call. Array power and load are shared
// evenly across channels until MBSU/DDCU distribution is modelled. No ROS.

namespace ssos_eps
{
namespace system
{

struct EpsInput
{
  double solar_flux_w_m2;      // solar flux at the station [W/m^2]
  bool in_eclipse;             // true in Earth's shadow
  double load_demand_w;        // total station load on the primary bus [W]
};

struct EpsState
{
  solar::SolarArrayState array;
  solar::SsuOutput ssu;
  std::vector<bcdu::BcduState> channels;
  std::vector<battery::BatteryOruState> orus;

  double generation_w;         // array terminal power [W]
  double available_power_w;    // SSU output available to the bus [W]
  double load_demand_w;        // requested load [W]
  double load_served_w;        // load actually supplied [W]
  double unserved_load_w;      // load not supplied [W]
  double battery_power_w;      // net battery terminal power, +discharge [W]
  double battery_charge_w;     // power into the batteries [W]
  double battery_discharge_w;  // power out of the batteries [W]
  double shunted_power_w;      // array power left unused [W]
  double conversion_loss_w;    // BCDU conversion losses [W]
  double power_balance_w;      // available_power_w - load_demand_w [W]
  double battery_soc;          // capacity-weighted bank SOC [0, 1]
  double min_oru_soc;          // lowest ORU SOC [0, 1]
  double stored_energy_wh;     // bank energy at open circuit [Wh]

  bool low_soc;                // bank SOC below threshold
  bool load_unserved;          // some load could not be supplied
  bool bcdu_fault;             // at least one BCDU faulted
  bool oru_failed;             // at least one ORU failed
  bool healthy;                // none of the above
  std::string status_message;
};

class EpsSystem
{
public:
  explicit EpsSystem(const EpsParameters & params = default_eps_parameters());
  EpsSystem(const EpsSystem &) = delete;
  EpsSystem & operator=(const EpsSystem &) = delete;

  /// Rebuild all channels and ORUs from the parameters and reset state.
  void reset();

  const EpsState & step(double dt, const EpsInput & input);

  /// Update parameters that do not change the channel/ORU topology.
  void set_parameters(const EpsParameters & params);
  const EpsParameters & parameters() const {return params_;}
  const EpsState & state() const {return state_;}

  int n_orus() const {return static_cast<int>(orus_.size());}
  int n_channels() const {return static_cast<int>(bcdus_.size());}

  void set_oru_failed(int index, bool failed);
  void set_bcdu_fault(int channel, bool faulted);
  void set_array_degradation(double factor);

private:
  void summarise(const EpsInput & input);

  EpsParameters params_;
  solar::SolarArrayModel array_;
  solar::SsuModel ssu_;
  std::vector<bcdu::BcduModel> bcdus_;
  std::vector<std::vector<battery::BatteryOruModel>> orus_by_channel_;
  std::vector<battery::BatteryOruModel *> orus_;
  EpsState state_{};
};

}  // namespace system
}  // namespace ssos_eps

#endif  // SSOS_EPS__SYSTEM__EPS_SYSTEM_HPP_
