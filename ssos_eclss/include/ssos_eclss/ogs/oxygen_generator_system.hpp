#ifndef SSOS_ECLSS__OGS__OXYGEN_GENERATOR_SYSTEM_HPP_
#define SSOS_ECLSS__OGS__OXYGEN_GENERATOR_SYSTEM_HPP_

#include <memory>

#include "ssos_eclss/ogs/electrolysis_stack_model.hpp"
#include "ssos_eclss/ogs/gas_separator_model.hpp"
#include "ssos_eclss/ogs/ogs_parameters.hpp"

// Top-level Oxygen Generation System. Inputs feedwater and electrical power;
// outputs O2 to the cabin and H2 to the Sabatier reactor. No ROS.

namespace ssos_eclss
{
namespace ogs
{

/// OGS outputs after a step.
struct OgsResult
{
  double o2_production_mol_s;   // O2 delivered to cabin [mol/s]
  double h2_production_mol_s;   // H2 delivered to Sabatier [mol/s]
  double o2_production_kg_day;  // O2 production [kg/day]
  double water_consumed_mol_s;  // feedwater consumed [mol/s]
  double water_consumed_kg_day; // feedwater consumed [kg/day]
  double stack_voltage;         // stack voltage [V]
  double stack_power_w;         // electrical power [W]
  double stack_temperature_k;   // stack temperature [K]
  double specific_energy_wh_kg; // energy per kg O2 [Wh/kg]
  bool feedwater_limited;       // true if feed water was the limiter
};

/// Oxygen Generation System orchestrator.
class OxygenGeneratorSystem
{
public:
  explicit OxygenGeneratorSystem(const OgsParameters & params = default_ogs_parameters());

  void reset(double temperature_k = 330.0);

  /// Advance the OGS by dt.
  /// @param dt                timestep [s]
  /// @param current_a         commanded stack current [A]
  /// @param available_water_mol_s feed water available [mol/s]
  OgsResult step(double dt, double current_a, double available_water_mol_s);

  /// Convenience: run at the configured operating current with ample feed.
  OgsResult step_nominal(double dt);

  void set_parameters(const OgsParameters & params);
  const OgsParameters & parameters() const { return params_; }

private:
  OgsParameters params_;
  std::unique_ptr<ElectrolysisStackModel> stack_;
  std::unique_ptr<GasSeparatorModel> separator_;
};

}  // namespace ogs
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__OGS__OXYGEN_GENERATOR_SYSTEM_HPP_
