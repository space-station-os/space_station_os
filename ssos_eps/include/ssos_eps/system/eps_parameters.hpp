#ifndef SSOS_EPS__SYSTEM__EPS_PARAMETERS_HPP_
#define SSOS_EPS__SYSTEM__EPS_PARAMETERS_HPP_

#include "ssos_eps/battery/battery_parameters.hpp"
#include "ssos_eps/bcdu/bcdu_parameters.hpp"
#include "ssos_eps/solar/solar_parameters.hpp"

// Complete generation-and-storage parameter set: the array feeds n_channels
// identical power channels, each with one BCDU and orus_per_channel batteries.

namespace ssos_eps
{
namespace system
{

struct EpsParameters
{
  int n_channels;              // independent power channels [-]
  int orus_per_channel;        // battery ORUs per channel [-]
  double low_soc_threshold;    // bank SOC below which EPS is unhealthy [-]
  solar::SolarParameters solar;
  battery::BatteryOruParams battery;
  bcdu::BcduParams bcdu;
};

EpsParameters default_eps_parameters();

}  // namespace system
}  // namespace ssos_eps

#endif  // SSOS_EPS__SYSTEM__EPS_PARAMETERS_HPP_
