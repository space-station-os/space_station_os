#ifndef SSOS_ECLSS__OGS__OGS_PARAMETERS_HPP_
#define SSOS_ECLSS__OGS__OGS_PARAMETERS_HPP_

// Parameters for the Oxygen Generation System (PEM water electrolysis).
// SI units. Factory functions provide ISS-OGA-class defaults.

namespace ssos_eclss
{
namespace ogs
{

/// PEM electrolysis cell electrochemistry/geometry.
struct CellParams
{
  double active_area_m2;       // membrane active area [m^2]
  double exchange_current_density;  // i0 [A/m^2] (lumped anode/cathode)
  double limiting_current_density;  // i_lim [A/m^2]
  double membrane_resistance;  // area-specific resistance [ohm*m^2]
  double charge_transfer_coeff;  // alpha [-]
  double reference_temp_k;     // reference temperature [K]
};

/// Electrolysis stack.
struct StackParams
{
  int n_cells;                 // number of series cells
  double thermal_mass_j_k;     // lumped stack thermal mass [J/K]
  double heat_loss_coeff_w_k;  // stack-to-coolant conductance [W/K]
};

/// Gas/water phase separator.
struct SeparatorParams
{
  double efficiency;           // separation efficiency [-]
  double carryover_fraction;   // liquid water carried with gas [-]
};

/// Operating set-points.
struct OgsOperating
{
  double stack_current_a;      // operating current [A]
  double feedwater_temp_k;     // feed water temperature [K]
  double coolant_temp_k;       // coolant temperature [K]
  double cell_pressure_pa;     // operating pressure [Pa]
};

/// Complete OGS parameter set.
struct OgsParameters
{
  CellParams cell;
  StackParams stack;
  SeparatorParams separator;
  OgsOperating operating;
};

CellParams default_cell_params();
StackParams default_stack_params();
SeparatorParams default_separator_params();
OgsOperating default_ogs_operating();
OgsParameters default_ogs_parameters();

}  // namespace ogs
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__OGS__OGS_PARAMETERS_HPP_
