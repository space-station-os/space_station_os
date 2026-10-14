#ifndef SSOS_EPS__BATTERY__BATTERY_PARAMETERS_HPP_
#define SSOS_EPS__BATTERY__BATTERY_PARAMETERS_HPP_

#include <vector>

// Parameters for one lithium-ion battery orbital replacement unit (ORU).
// SI units unless the field name says otherwise. The factory function gives an
// ISS Li-ion ORU-class pack: 30 cells in series, 134 Ah.

namespace ssos_eps
{
namespace battery
{

struct BatteryOruParams
{
  int n_series_cells;                // cells in series [-]
  double capacity_ah;                // rated capacity at beginning of life [Ah]
  double state_of_health;            // usable fraction of rated capacity [-]
  std::vector<double> soc_points;    // SOC breakpoints for the OCV table [-]
  std::vector<double> cell_ocv_v;    // cell open-circuit voltage at soc_points [V]
  double internal_resistance_ohm;    // pack series resistance [ohm]
  double coulombic_efficiency;       // charge accepted / charge supplied [-]
  double max_charge_current_a;       // charge current limit [A]
  double max_discharge_current_a;    // discharge current limit [A]
  double max_cell_voltage_v;         // constant-voltage charge limit per cell [V]
  double min_cell_voltage_v;         // discharge cut-off per cell [V]
  double thermal_mass_j_k;           // lumped heat capacity [J/K]
  double heat_transfer_w_k;          // conductance to the cold plate [W/K]
  double coldplate_temp_c;           // cold plate temperature [C]
  double max_temp_c;                 // over-temperature limit [C]
  double initial_soc;                // state of charge at reset [-]
  double initial_temp_c;             // temperature at reset [C]
};

BatteryOruParams default_battery_oru_params();

}  // namespace battery
}  // namespace ssos_eps

#endif  // SSOS_EPS__BATTERY__BATTERY_PARAMETERS_HPP_
