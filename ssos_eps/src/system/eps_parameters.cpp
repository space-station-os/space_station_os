#include "ssos_eps/system/eps_parameters.hpp"

namespace ssos_eps
{
namespace system
{

std::vector<loads::LoadSpec> default_baseline_loads()
{
  // 30 kW of station load in priority classes: 0 = vehicle-critical,
  // 1 = life support and communications, 2 = crew, 3 = payloads.
  return {
    {"gnc_avionics", 3000.0, 0, -1},
    {"cdh_computers", 2500.0, 0, -1},
    {"thermal_pumps", 4000.0, 0, -1},
    {"eclss_ars", 1500.0, 0, -1},
    {"eclss_ogs", 3500.0, 1, -1},
    {"eclss_wrs", 1200.0, 1, -1},
    {"comms", 1800.0, 1, -1},
    {"crew_systems", 3500.0, 2, -1},
    {"lab_payloads_a", 4500.0, 3, -1},
    {"lab_payloads_b", 4500.0, 3, -1},
  };
}

EpsParameters default_eps_parameters()
{
  EpsParameters p{};
  p.n_channels = 12;
  p.orus_per_channel = 2;
  p.n_mbsu = 4;
  p.channel_mbsu = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3};
  p.ddcu_mbsu = {0, 0, 1, 1, 2, 2, 3, 3};
  p.low_soc_threshold = 0.20;
  p.min_state_of_health = 0.80;
  p.solar = solar::default_solar_parameters();
  p.battery = battery::default_battery_oru_params();
  p.bcdu = bcdu::default_bcdu_params();
  p.mbsu = mbsu::default_mbsu_params();
  p.ddcu = ddcu::default_ddcu_params();
  p.baseline_loads = default_baseline_loads();
  p.demand_scale = 1.0;
  return p;
}

}  // namespace system
}  // namespace ssos_eps
