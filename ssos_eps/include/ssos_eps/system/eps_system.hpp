#ifndef SSOS_EPS__SYSTEM__EPS_SYSTEM_HPP_
#define SSOS_EPS__SYSTEM__EPS_SYSTEM_HPP_

#include <string>
#include <vector>

#include "ssos_eps/battery/battery_oru_model.hpp"
#include "ssos_eps/bcdu/bcdu_model.hpp"
#include "ssos_eps/ddcu/ddcu_model.hpp"
#include "ssos_eps/loads/load_registry.hpp"
#include "ssos_eps/mbsu/mbsu_model.hpp"
#include "ssos_eps/solar/solar_array_model.hpp"
#include "ssos_eps/solar/ssu_model.hpp"
#include "ssos_eps/system/eps_parameters.hpp"

// Top-level EPS: generation, storage and distribution. Inputs the environment
// (solar flux, eclipse); loads are registered on the system. Each call steps
// array -> SSU, works out what every channel can supply, routes DDCU demand
// through the MBSUs onto the channels (shedding the least critical loads when
// a DDCU is over its rating or an MBSU cannot cover its demand), then steps
// the BCDUs/batteries and the DDCUs. No ROS.

namespace ssos_eps
{
namespace system
{

struct EpsInput
{
  double solar_flux_w_m2;      // solar flux at the station [W/m^2]
  bool in_eclipse;             // true in Earth's shadow
};

/// Outcome of a load request or release.
struct LoadResult
{
  bool accepted;
  std::string message;
  int ddcu;                    // DDCU the load is on (-1 if rejected)
  double granted_w;            // power allocated [W]
  double supply_voltage_v;     // DDCU regulated output [V]
};

struct EpsState
{
  solar::SolarArrayState array;
  solar::SsuOutput ssu;
  std::vector<bcdu::BcduState> channels;
  std::vector<battery::BatteryOruState> orus;
  std::vector<mbsu::MbsuState> mbsus;
  std::vector<ddcu::DdcuState> ddcus;
  std::vector<loads::LoadState> loads;

  double generation_w;         // array terminal power [W]
  double available_power_w;    // SSU output available to the bus [W]
  double supply_capability_w;  // what all channels could deliver this step [W]
  double primary_load_w;       // power routed from the channels to the MBSUs [W]
  double load_demand_w;        // requested by all loads [W]
  double load_served_w;        // delivered to loads by the DDCUs [W]
  double unserved_load_w;      // load_demand_w - load_served_w [W]
  int shed_loads;              // loads switched off this step
  double distribution_loss_w;  // DDCU conversion losses [W]
  double battery_power_w;      // net battery terminal power, +discharge [W]
  double battery_charge_w;     // power into the batteries [W]
  double battery_discharge_w;  // power out of the batteries [W]
  double shunted_power_w;      // array power left unused [W]
  double conversion_loss_w;    // BCDU conversion losses [W]
  double power_balance_w;      // available_power_w - primary_load_w [W]
  double battery_soc;          // capacity-weighted bank SOC [0, 1]
  double min_oru_soc;          // lowest ORU SOC [0, 1]
  double stored_energy_wh;     // bank energy at open circuit [Wh]

  bool low_soc;                // bank SOC below threshold
  bool load_unserved;          // some requested load not delivered
  bool load_shed;              // at least one load switched off
  bool bcdu_fault;             // at least one BCDU faulted
  bool oru_failed;             // at least one ORU failed
  bool oru_overheat;           // at least one ORU above its temperature limit
  bool ddcu_fault;             // a DDCU with loads is tripped or off
  bool ddcu_overload;          // a DDCU's requested load exceeds its rating
  bool mbsu_unstable;          // an MBSU has no usable bus
  bool redundancy_lost;        // an MBSU is down to fewer than its redundant sources
  bool array_degraded;         // array output below beginning of life
  bool battery_degraded;       // battery state of health below the minimum
  bool healthy;                // none of the above
  std::string status_message;
};

class EpsSystem
{
public:
  explicit EpsSystem(const EpsParameters & params = default_eps_parameters());
  EpsSystem(const EpsSystem &) = delete;
  EpsSystem & operator=(const EpsSystem &) = delete;

  /// Rebuild all units from the parameters, reload the baseline loads and
  /// reset state.
  void reset();

  const EpsState & step(double dt, const EpsInput & input);

  /// Update parameters that do not change the topology. Registered loads and
  /// battery state are kept.
  void set_parameters(const EpsParameters & params);
  const EpsParameters & parameters() const {return params_;}
  const EpsState & state() const {return state_;}

  int n_orus() const {return static_cast<int>(orus_.size());}
  int n_channels() const {return static_cast<int>(bcdus_.size());}
  int n_mbsu() const {return static_cast<int>(mbsus_.size());}
  int n_ddcu() const {return static_cast<int>(ddcus_.size());}

  /// Admit or update a load. preferred_ddcu = -1 picks the DDCU with the most
  /// headroom. Rejected if the request is invalid, no available DDCU has the
  /// headroom, or the supply margin from the last step cannot carry it.
  LoadResult request_load(const std::string & id, double power_w, int priority,
                          int preferred_ddcu = -1);
  LoadResult release_load(const std::string & id);
  const loads::LoadRegistry & loads() const {return registry_;}
  void set_demand_scale(double scale);

  void set_oru_failed(int index, bool failed);
  void set_bcdu_fault(int channel, bool faulted);
  void set_array_degradation(double factor);
  /// Set every ORU's state of charge, keeping failures and temperature.
  void set_battery_soc(double soc);
  void set_ddcu_fault(int index, bool faulted);
  void set_mbsu_fault(int index, bool faulted);
  /// Open the feeder from a channel to its MBSU.
  void set_channel_offline(int channel, bool offline);

private:
  bool ddcu_admits(int d) const;
  int mbsu_of_ddcu(int d) const;
  void load_baseline();
  void summarise();

  EpsParameters params_;
  solar::SolarArrayModel array_;
  solar::SsuModel ssu_;
  std::vector<bcdu::BcduModel> bcdus_;
  std::vector<std::vector<battery::BatteryOruModel>> orus_by_channel_;
  std::vector<battery::BatteryOruModel *> orus_;
  std::vector<mbsu::MbsuModel> mbsus_;
  std::vector<ddcu::DdcuModel> ddcus_;
  std::vector<bool> channel_offline_;
  loads::LoadRegistry registry_;
  EpsState state_{};
  bool stepped_{false};
};

}  // namespace system
}  // namespace ssos_eps

#endif  // SSOS_EPS__SYSTEM__EPS_SYSTEM_HPP_
