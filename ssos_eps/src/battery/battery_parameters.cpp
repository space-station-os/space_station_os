#include "ssos_eps/battery/battery_parameters.hpp"

namespace ssos_eps
{
namespace battery
{

BatteryOruParams default_battery_oru_params()
{
  BatteryOruParams p{};
  p.n_series_cells = 30;
  p.capacity_ah = 134.0;
  p.state_of_health = 1.0;
  // NMC-type cell open-circuit voltage curve.
  p.soc_points = {0.0, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00};
  p.cell_ocv_v = {3.00, 3.30, 3.45, 3.55, 3.62, 3.67, 3.72, 3.80, 3.88, 3.95, 4.03, 4.10};
  p.internal_resistance_ohm = 0.05;
  p.coulombic_efficiency = 0.99;
  p.max_charge_current_a = 40.0;
  p.max_discharge_current_a = 100.0;
  p.max_cell_voltage_v = 4.10;
  p.min_cell_voltage_v = 3.00;
  p.thermal_mass_j_k = 2.0e5;
  p.heat_transfer_w_k = 20.0;
  p.coldplate_temp_c = 10.0;
  p.max_temp_c = 35.0;
  p.initial_soc = 0.80;
  p.initial_temp_c = 10.0;
  return p;
}

}  // namespace battery
}  // namespace ssos_eps
