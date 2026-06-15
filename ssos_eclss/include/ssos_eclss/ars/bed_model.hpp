#ifndef SSOS_ECLSS__ARS__BED_MODEL_HPP_
#define SSOS_ECLSS__ARS__BED_MODEL_HPP_

#include <cstddef>
#include <vector>

#include "ssos_eclss/ars/adsorption_isotherm.hpp"
#include "ssos_eclss/ars/ars_parameters.hpp"

// 1D finite-volume packed-bed adsorber model. Solves coupled gas/solid mass and
// energy balances with a Linear-Driving-Force sorption rate, upwind advection,
// axial dispersion and Ergun pressure drop. No ROS, no external deps.

namespace ssos_eclss
{
namespace ars
{

/// Operating mode of a bed within the cycle.
enum class BedMode
{
  ADSORBING,       // forward process flow, capturing CO2/H2O
  AIR_SAVE,        // recovering cabin air before vacuum (low flow)
  DESORBING,       // heated regeneration toward vacuum
  VACUUM,          // vacuum desorption tail
  IDLE,            // no flow, holding state
  REPRESSURIZING   // refilling from vacuum back to cabin pressure
};

/// Inlet boundary condition presented to a bed for one step.
struct BedInlet
{
  double velocity_superficial;  // superficial gas velocity [m/s]
  double c_co2;                 // inlet CO2 concentration [mol/m^3]
  double c_h2o;                 // inlet H2O concentration [mol/m^3]
  double temperature_k;         // inlet gas temperature [K]
  double pressure_pa;           // operating pressure [Pa]
};

/// Aggregate outputs after a step.
struct BedOutputs
{
  double outlet_c_co2;       // outlet CO2 concentration [mol/m^3]
  double outlet_c_h2o;       // outlet H2O concentration [mol/m^3]
  double outlet_temperature; // outlet gas temperature [K]
  double max_solid_temp;     // peak solid temperature [K]
  double mean_solid_temp;    // mean solid temperature [K]
  double co2_loading_mol;    // total CO2 held on solid [mol]
  double h2o_loading_mol;    // total H2O held on solid [mol]
  double pressure_drop_pa;   // bed pressure drop [Pa]
  double co2_capture_rate;   // instantaneous CO2 captured [mol/s]
};

/// One packed adsorber bed.
class BedModel
{
public:
  /// @param geom      bed geometry
  /// @param thermal   bed thermal parameters
  /// @param co2       CO2 Toth isotherm (use even for desiccant; sees ~0 capacity)
  /// @param h2o       H2O Toth isotherm
  /// @param ldf       LDF mass-transfer coefficients
  /// @param is_desiccant true for a desiccant (water-only) bed
  BedModel(const BedGeometry & geom, const BedThermal & thermal,
           const TothIsotherm & co2, const TothIsotherm & h2o,
           const LDFParams & ldf, bool is_desiccant);

  /// Reset all state to a uniform clean, ambient condition.
  /// @param temperature_k initial temperature [K]
  void reset(double temperature_k);

  /// Pre-load the bed to equilibrium with given inlet partial pressures
  /// (useful to start near cyclic steady state).
  void equilibrate(double pp_co2_pa, double pp_h2o_pa, double temperature_k);

  /// Advance the bed by @p dt seconds.
  /// @param dt          macro timestep [s]
  /// @param inlet       inlet boundary condition
  /// @param mode        operating mode
  /// @param heater_power_w heater power applied (DESORBING/AIR_SAVE) [W]
  /// @return aggregate outputs after the step
  BedOutputs step(double dt, const BedInlet & inlet, BedMode mode,
                  double heater_power_w);

  /// Current aggregate outputs without advancing time.
  BedOutputs snapshot(const BedInlet & inlet) const;

  // ---- State accessors (for tests / telemetry) ----
  std::size_t n_cells() const { return n_; }
  const std::vector<double> & co2_loading() const { return q_co2_; }
  const std::vector<double> & h2o_loading() const { return q_h2o_; }
  const std::vector<double> & solid_temperature() const { return ts_; }
  const std::vector<double> & gas_temperature() const { return tg_; }
  double total_co2_loading_mol() const;
  double total_h2o_loading_mol() const;
  double mean_solid_temperature() const;
  double max_solid_temperature() const;

private:
  // Advance all state fields by one stable substep of size h. Mass, solid energy
  // and sorption are integrated explicitly; the (stiff) gas-energy gas-solid and
  // wall exchange is integrated semi-implicitly for unconditional stability.
  void integrate_substep(double h, const BedInlet & inlet, BedMode mode,
                         double heater_power_w);

  double bed_pressure_drop(const BedInlet & inlet) const;

  BedGeometry geom_;
  BedThermal thermal_;
  TothIsotherm co2_iso_;
  TothIsotherm h2o_iso_;
  LDFParams ldf_;
  bool is_desiccant_;

  std::size_t n_;
  double dz_;
  double rho_bulk_;  // (1-eps) * particle density [kg/m^3]

  // State fields (size n_)
  std::vector<double> c_co2_;  // gas CO2 [mol/m^3]
  std::vector<double> c_h2o_;  // gas H2O [mol/m^3]
  std::vector<double> q_co2_;  // solid CO2 [mol/kg]
  std::vector<double> q_h2o_;  // solid H2O [mol/kg]
  std::vector<double> tg_;     // gas temperature [K]
  std::vector<double> ts_;     // solid temperature [K]
};

}  // namespace ars
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__ARS__BED_MODEL_HPP_
