#ifndef SSOS_EPS__DDCU__DDCU_PARAMETERS_HPP_
#define SSOS_EPS__DDCU__DDCU_PARAMETERS_HPP_

// Parameters for one DC-to-DC converter unit (DDCU) stepping primary bus power
// down to the regulated secondary bus. Factory defaults are DDCU-class values
// matching the legacy space_station_eps DDCU (124.5 V output, 115-173 V input).

namespace ssos_eps
{
namespace ddcu
{

struct DdcuParams
{
  double input_min_v;             // lowest usable primary voltage [V]
  double input_max_v;             // highest usable primary voltage [V]
  double output_voltage_v;        // no-load regulated output [V]
  double regulation_droop_v;      // output drop at rated power [V]
  double rated_power_w;           // continuous output rating [W]
  double efficiency;              // conversion efficiency at load [-]
  double no_load_loss_w;          // standby loss while regulating [W]
  double thermal_mass_j_k;        // lumped heat capacity [J/K]
  double heat_transfer_w_k;       // conductance to the cold plate [W/K]
  double coldplate_temp_c;        // cold plate temperature [C]
  double trip_temp_c;             // over-temperature trip [C]
  double reset_temp_c;            // temperature at which a thermal trip clears [C]
  double initial_temp_c;          // temperature at reset [C]
};

DdcuParams default_ddcu_params();

}  // namespace ddcu
}  // namespace ssos_eps

#endif  // SSOS_EPS__DDCU__DDCU_PARAMETERS_HPP_
