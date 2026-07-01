#ifndef SSOS_ECLSS__ARS__FOUR_BED_SYSTEM_HPP_
#define SSOS_ECLSS__ARS__FOUR_BED_SYSTEM_HPP_

#include <array>
#include <memory>

#include "ssos_eclss/ars/adsorption_isotherm.hpp"
#include "ssos_eclss/ars/air_save_pump_model.hpp"
#include "ssos_eclss/ars/ars_parameters.hpp"
#include "ssos_eclss/ars/bed_model.hpp"
#include "ssos_eclss/ars/blower_model.hpp"
#include "ssos_eclss/ars/cycle_state_machine.hpp"
#include "ssos_eclss/ars/precooler_model.hpp"
#include "ssos_eclss/ars/valve_model.hpp"

// Top-level Air Revitalization System (4BMS). Owns two desiccant beds, two
// adsorbent beds, the precooler, blower, air-save pump, valves and the cycle
// state machine. A single step() advances the entire assembly. This is the
// object the ARS ROS node drives. No ROS, no external deps.

namespace ssos_eclss
{
namespace ars
{

/// Cabin (process-air inlet) conditions presented to the ARS each step.
struct CabinConditions
{
  double co2_partial_pressure_pa;  // cabin ppCO2 [Pa]
  double h2o_partial_pressure_pa;  // cabin water-vapour partial pressure [Pa]
  double temperature_k;            // cabin air temperature [K]
  double total_pressure_pa;        // cabin total pressure [Pa]
};

/// ARS outputs after a step.
struct ArsResult
{
  double co2_removal_rate_kg_s;    // net CO2 removed from the cabin [kg/s]
  double co2_removal_rate_kg_day;  // net CO2 removed [kg/day]
  double scrubbed_co2_pp_pa;       // CO2 partial pressure of return air [Pa]
  double scrubbed_h2o_pp_pa;       // H2O partial pressure of return air [Pa]
  double return_air_temp_k;        // return air temperature [K]
  double system_pressure_drop_pa;  // total flow-path pressure drop [Pa]
  double blower_flow_scfm;         // delivered process flow [SCFM]
  double blower_power_w;           // blower shaft power [W]
  double precooler_exit_temp_k;    // precooler exit air temperature [K]
  double max_bed_temp_k;           // hottest solid temperature in any bed [K]
  double co2_desorbed_rate_kg_s;   // CO2 released by the regenerating bed [kg/s]
  double air_saved_rate_kg_s;      // cabin air recovered by air-save pump [kg/s]
  int adsorbing_train;             // active train index (0/1)
};

/// Four-Bed Molecular Sieve CO2 removal assembly.
class FourBedSystem
{
public:
  explicit FourBedSystem(const ArsParameters & params = default_ars_parameters());

  /// Re-initialise all beds to ambient and restart the cycle.
  void reset(double temperature_k = 295.0);

  /// Advance the whole assembly by dt seconds under given cabin conditions.
  ArsResult step(double dt, const CabinConditions & cabin);

  /// Design-point steady CO2 removal [kg/day] from the configured operating
  /// conditions and net capture efficiency (used for validation).
  double design_co2_removal_kg_day() const;

  /// Update parameters live (e.g. from the ROS parameter bridge). Geometry and
  /// cell-count changes require reconstruct(); this updates the tunable fields.
  void set_parameters(const ArsParameters & params);

  /// Fully rebuild the bed objects (needed when geometry / n_cells change).
  void reconstruct();

  const ArsParameters & parameters() const { return params_; }
  const CycleStateMachine & cycle() const { return *cycle_; }

  // Bed accessors (0/1 = train, desiccant or adsorbent).
  BedModel & desiccant_bed(int i) { return *desiccant_[i]; }
  BedModel & adsorbent_bed(int i) { return *adsorbent_[i]; }

private:
  double inlet_co2_mass_rate(const CabinConditions & cabin, double flow_m3s) const;

  ArsParameters params_;
  TothIsotherm co2_iso_;
  TothIsotherm h2o_iso_;
  TothIsotherm h2o_silica_iso_;

  std::array<std::unique_ptr<BedModel>, 2> desiccant_;
  std::array<std::unique_ptr<BedModel>, 2> adsorbent_;
  std::unique_ptr<PrecoolerModel> precooler_;
  std::unique_ptr<BlowerModel> blower_;
  std::unique_ptr<AirSavePumpModel> air_save_pump_;
  std::unique_ptr<ValveModel> valve_;
  std::unique_ptr<CycleStateMachine> cycle_;

  double system_resistance_{0.0};  // Pa/(m^3/s)^2
};

}  // namespace ars
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__ARS__FOUR_BED_SYSTEM_HPP_
