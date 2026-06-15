#ifndef SSOS_ECLSS__OGS__ELECTROLYSIS_STACK_MODEL_HPP_
#define SSOS_ECLSS__OGS__ELECTROLYSIS_STACK_MODEL_HPP_

#include "ssos_eclss/ogs/electrolysis_cell_model.hpp"
#include "ssos_eclss/ogs/ogs_parameters.hpp"

// Series stack of PEM cells with a lumped thermal model. No ROS.

namespace ssos_eclss
{
namespace ogs
{

/// Aggregate stack operating point.
struct StackState
{
  double total_voltage;       // stack voltage [V]
  double current_a;           // stack current [A]
  double o2_production_mol_s; // total O2 [mol/s]
  double h2_production_mol_s; // total H2 [mol/s]
  double power_w;             // total electrical power [W]
  double waste_heat_w;        // heat above thermoneutral [W]
  double temperature_k;       // stack temperature [K]
  double efficiency;          // voltage efficiency [-]
};

/// PEM electrolysis stack.
class ElectrolysisStackModel
{
public:
  ElectrolysisStackModel(const StackParams & stack, const CellParams & cell);

  /// Reset the stack temperature.
  void reset(double temperature_k);

  /// Advance the stack thermal state and compute the operating point.
  /// @param dt          timestep [s]
  /// @param current_a   stack current [A]
  /// @param pressure_pa operating pressure [Pa]
  /// @param coolant_temp_k coolant temperature [K]
  StackState step(double dt, double current_a, double pressure_pa,
                  double coolant_temp_k);

  double temperature_k() const { return temperature_k_; }
  const StackParams & params() const { return stack_; }
  void set_params(const StackParams & s) { stack_ = s; }
  void set_cell_params(const CellParams & c) { cell_.set_params(c); }

private:
  StackParams stack_;
  ElectrolysisCellModel cell_;
  double temperature_k_{330.0};
};

}  // namespace ogs
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__OGS__ELECTROLYSIS_STACK_MODEL_HPP_
