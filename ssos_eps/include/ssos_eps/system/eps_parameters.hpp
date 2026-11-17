#ifndef SSOS_EPS__SYSTEM__EPS_PARAMETERS_HPP_
#define SSOS_EPS__SYSTEM__EPS_PARAMETERS_HPP_

#include <vector>

#include "ssos_eps/battery/battery_parameters.hpp"
#include "ssos_eps/bcdu/bcdu_parameters.hpp"
#include "ssos_eps/ddcu/ddcu_parameters.hpp"
#include "ssos_eps/loads/load_registry.hpp"
#include "ssos_eps/mbsu/mbsu_parameters.hpp"
#include "ssos_eps/solar/solar_parameters.hpp"

// Complete EPS parameter set. The array feeds n_channels identical power
// channels, each with one BCDU and orus_per_channel batteries. Each channel is
// wired to one MBSU (channel_mbsu), each DDCU hangs off one MBSU (ddcu_mbsu,
// whose length is the DDCU count), and the baseline loads sit on the DDCUs.

namespace ssos_eps
{
namespace system
{

struct EpsParameters
{
  int n_channels;              // independent power channels [-]
  int orus_per_channel;        // battery ORUs per channel [-]
  int n_mbsu;                  // main bus switching units [-]
  std::vector<int> channel_mbsu;  // MBSU fed by each channel (-1 = unconnected)
  std::vector<int> ddcu_mbsu;     // MBSU feeding each DDCU
  double low_soc_threshold;    // bank SOC below which EPS is unhealthy [-]
  double min_state_of_health; // battery state of health below which EPS is degraded [-]
  solar::SolarParameters solar;
  battery::BatteryOruParams battery;
  bcdu::BcduParams bcdu;
  mbsu::MbsuParams mbsu;
  ddcu::DdcuParams ddcu;
  std::vector<loads::LoadSpec> baseline_loads;
  double demand_scale;         // multiplier on every load's nominal power [-]
};

std::vector<loads::LoadSpec> default_baseline_loads();
EpsParameters default_eps_parameters();

}  // namespace system
}  // namespace ssos_eps

#endif  // SSOS_EPS__SYSTEM__EPS_PARAMETERS_HPP_
