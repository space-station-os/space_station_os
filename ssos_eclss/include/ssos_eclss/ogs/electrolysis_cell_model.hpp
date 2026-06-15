#ifndef SSOS_ECLSS__OGS__ELECTROLYSIS_CELL_MODEL_HPP_
#define SSOS_ECLSS__OGS__ELECTROLYSIS_CELL_MODEL_HPP_

#include "ssos_eclss/ogs/ogs_parameters.hpp"

// Single PEM electrolysis cell: 2 H2O -> 2 H2 + O2. Computes cell voltage as
// the reversible (Nernst) potential plus activation (Tafel), ohmic and
// concentration overpotentials, and the Faradaic gas production. No ROS.

namespace ssos_eclss
{
namespace ogs
{

/// Operating point of a single cell.
struct CellState
{
  double voltage;            // total cell voltage [V]
  double reversible_voltage; // Nernst potential [V]
  double activation_overpotential;  // [V]
  double ohmic_overpotential;        // [V]
  double concentration_overpotential;  // [V]
  double o2_production_mol_s; // O2 generated [mol/s]
  double h2_production_mol_s; // H2 generated [mol/s]
  double power_w;             // electrical power [W]
  double efficiency;          // thermoneutral/voltage efficiency [-]
};

/// PEM electrolysis cell model.
class ElectrolysisCellModel
{
public:
  explicit ElectrolysisCellModel(const CellParams & params);

  /// Reversible cell voltage [V] via Nernst at temperature and pressure.
  double reversible_voltage(double temperature_k, double pressure_pa) const;

  /// Solve the cell at a given current and conditions.
  /// @param current_a    cell current [A]
  /// @param temperature_k cell temperature [K]
  /// @param pressure_pa  operating pressure [Pa]
  CellState solve(double current_a, double temperature_k, double pressure_pa) const;

  void set_params(const CellParams & p) { params_ = p; }
  const CellParams & params() const { return params_; }

private:
  CellParams params_;
};

}  // namespace ogs
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__OGS__ELECTROLYSIS_CELL_MODEL_HPP_
