#ifndef SSOS_ECLSS__ARS__ARS_PARAMETERS_HPP_
#define SSOS_ECLSS__ARS__ARS_PARAMETERS_HPP_

#include <cstddef>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Parameter structs for the 4-Bed Molecular Sieve (4BMS) CO2 removal assembly.
// All values default to the 4BCO2 EDU configuration described in
// Peters, Cmarik, Knox, "4BCO2 EDU Performance", ICES-2021-313 (2021),
// and the supporting isotherm-material papers (Knox & Cmarik 2019).
//
// Factory functions return defaults only; the physics never hardcodes these.
// SI units throughout unless noted.

namespace ssos_eclss
{
namespace ars
{

/// Single-species Toth isotherm coefficients.
/// q*(P,T) = q_m(T) b(T) P / [1 + (b(T) P)^t]^(1/t)
/// q_m(T)  = q_m0 exp[chi (1 - T/T_ref)]
/// b(T)    = b0 exp[(dH / (R T_ref)) (T_ref/T - 1)]
/// t(T)    = t0 + alpha (1 - T_ref/T)
struct TothParams
{
  double q_m0;     // saturation loading at T_ref [mol/kg]
  double chi;      // temperature exponent for q_m [-]
  double b0;       // affinity constant at T_ref [1/Pa]
  double dH;       // isosteric heat of adsorption (positive) [J/mol]
  double t0;       // heterogeneity exponent at T_ref [-] (0,1]
  double alpha;    // temperature dependence of t [-]
  double T_ref;    // reference temperature [K]
};

/// Linear-Driving-Force mass-transfer coefficients [1/s].
struct LDFParams
{
  double k_co2;    // CO2 LDF rate [1/s]
  double k_h2o;    // H2O LDF rate [1/s]
};

/// Packed-bed geometry and bulk properties.
struct BedGeometry
{
  double length;             // bed length [m]
  double diameter;           // bed internal diameter [m]
  double voidage;            // interparticle void fraction [-]
  double particle_diameter;  // sorbent pellet diameter [m]
  double sorbent_density;    // sorbent particle (skeletal+pore) density [kg/m^3]
  std::size_t n_cells;       // axial finite-volume cells [-]

  /// Cross-sectional area [m^2].
  double cross_area() const { return M_PI * 0.25 * diameter * diameter; }
  /// Total bed volume [m^3].
  double volume() const { return cross_area() * length; }
  /// Mass of sorbent in the bed [kg] (solid fraction times particle density).
  double sorbent_mass() const { return volume() * (1.0 - voidage) * sorbent_density; }
};

/// Thermal model parameters for a bed.
struct BedThermal
{
  double sorbent_cp;          // solid specific heat [J/(kg*K)]
  double wall_htc;            // gas-to-wall U [W/(m^2*K)]
  double wall_area_per_vol;   // wall area per bed volume [m^2/m^3]
  double wall_temp;           // wall/ambient temperature [K]
  double axial_thermal_cond;  // effective axial conductivity k_ax [W/(m*K)]
  double gas_thermal_cond;    // gas thermal conductivity [W/(m*K)]
};

/// Regeneration heater parameters.
struct HeaterParams
{
  double total_power_w;     // power with all 19 elements (desorption) [W]
  double central_power_w;   // power with central 7 elements (air-save) [W]
  double max_temp_k;        // heater control setpoint [K]
};

/// Half-cycle timing (10-60-10 sequence), seconds.
struct CycleTiming
{
  double air_save_s;   // air-save (cabin air recovery) [s]
  double adsorb_s;     // active adsorption [s]
  double vacuum_s;     // vacuum desorption tail [s]

  /// Total half-cycle duration [s].
  double half_cycle_s() const { return air_save_s + adsorb_s + vacuum_s; }
};

/// Operating / boundary conditions.
struct OperatingConditions
{
  double inlet_flow_scfm;     // process air flow [SCFM]
  double inlet_ppco2_torr;    // cabin CO2 partial pressure [torr]
  double inlet_dewpoint_k;    // process air dew point [K]
  double cabin_temp_k;        // process air temperature [K]
  double cabin_pressure_pa;   // total pressure [Pa]
  double ltl_inlet_temp_k;    // low-temperature-loop coolant inlet [K]
  double ltl_flow_gpm;        // coolant flow [gpm]
  double vacuum_pressure_pa;  // desorption vacuum pressure [Pa]
};

/// Lumped system-level loss factor capturing air-save inefficiency and bed
/// holdup loss that the cell-resolved model does not individually track.
/// The paper reports holdup loss 8-12% and net efficiency 0.82-0.84.
struct SystemEfficiency
{
  double capture_efficiency;  // net/gross CO2 capture [-]
  double holdup_loss;         // fraction of cabin air lost to vacuum [-]
};

/// Complete ARS parameter set.
struct ArsParameters
{
  BedGeometry desiccant_bed;
  BedGeometry adsorbent_bed;
  BedThermal desiccant_thermal;
  BedThermal adsorbent_thermal;
  TothParams co2_on_13x;
  TothParams h2o_on_13x;
  TothParams h2o_on_silica;
  LDFParams desiccant_ldf;
  LDFParams adsorbent_ldf;
  HeaterParams heater;
  CycleTiming cycle;
  OperatingConditions operating;
  SystemEfficiency efficiency;
};

// ---- Factory functions: 4BCO2 EDU defaults ----
TothParams default_co2_on_13x();
TothParams default_h2o_on_13x();
TothParams default_h2o_on_silica();
LDFParams default_desiccant_ldf();
LDFParams default_adsorbent_ldf();
BedGeometry default_desiccant_geometry();
BedGeometry default_adsorbent_geometry();
BedThermal default_desiccant_thermal();
BedThermal default_adsorbent_thermal();
HeaterParams default_heater();
CycleTiming default_cycle_timing();
OperatingConditions default_operating_conditions();
SystemEfficiency default_system_efficiency();
ArsParameters default_ars_parameters();

}  // namespace ars
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__ARS__ARS_PARAMETERS_HPP_
