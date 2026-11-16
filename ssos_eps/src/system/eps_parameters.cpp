#include "ssos_eps/system/eps_parameters.hpp"

namespace ssos_eps
{
namespace system
{

EpsParameters default_eps_parameters()
{
  EpsParameters p{};
  p.n_channels = 12;
  p.orus_per_channel = 2;
  p.low_soc_threshold = 0.20;
  p.solar = solar::default_solar_parameters();
  p.battery = battery::default_battery_oru_params();
  p.bcdu = bcdu::default_bcdu_params();
  return p;
}

}  // namespace system
}  // namespace ssos_eps
