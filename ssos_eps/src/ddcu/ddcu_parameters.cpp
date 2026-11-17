#include "ssos_eps/ddcu/ddcu_parameters.hpp"

namespace ssos_eps
{
namespace ddcu
{

DdcuParams default_ddcu_params()
{
  DdcuParams p{};
  p.input_min_v = 115.0;
  p.input_max_v = 173.0;
  p.output_voltage_v = 124.5;
  p.regulation_droop_v = 1.5;
  p.rated_power_w = 6250.0;
  p.efficiency = 0.96;
  p.no_load_loss_w = 50.0;
  p.thermal_mass_j_k = 2.0e4;
  p.heat_transfer_w_k = 8.0;
  p.coldplate_temp_c = 18.0;
  p.trip_temp_c = 70.0;
  p.reset_temp_c = 55.0;
  p.initial_temp_c = 20.0;
  return p;
}

}  // namespace ddcu
}  // namespace ssos_eps
