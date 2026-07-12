#include "ssos_eclss/ogs/electrolysis_stack_model.hpp"

#include <algorithm>

namespace ssos_eclss
{
namespace ogs
{

namespace
{
constexpr double kThermoneutral = 1.481;  // V
}

ElectrolysisStackModel::ElectrolysisStackModel(const StackParams & stack,
                                               const CellParams & cell)
: stack_(stack), cell_(cell), temperature_k_(330.0)
{}

void ElectrolysisStackModel::reset(double temperature_k)
{
  temperature_k_ = temperature_k;
}

StackState ElectrolysisStackModel::step(double dt, double current_a,
                                        double pressure_pa, double coolant_temp_k)
{
  const CellState cell = cell_.solve(current_a, temperature_k_, pressure_pa);
  const double n = static_cast<double>(std::max(stack_.n_cells, 1));

  StackState s{};
  s.current_a = current_a;
  s.total_voltage = cell.voltage * n;
  s.o2_production_mol_s = cell.o2_production_mol_s * n;
  s.h2_production_mol_s = cell.h2_production_mol_s * n;
  s.power_w = cell.power_w * n;
  s.efficiency = cell.efficiency;

  // Waste heat = (V_cell - V_thermoneutral) * I per cell, when V > V_tn.
  const double waste_per_cell = std::max(0.0, cell.voltage - kThermoneutral) * current_a;
  s.waste_heat_w = waste_per_cell * n;

  // Lumped thermal update: dT/dt = (Q_waste - UA (T - T_cool)) / C.
  const double q_cool = stack_.heat_loss_coeff_w_k * (temperature_k_ - coolant_temp_k);
  const double dT = (s.waste_heat_w - q_cool) / std::max(stack_.thermal_mass_j_k, 1.0);
  temperature_k_ += dT * dt;
  s.temperature_k = temperature_k_;
  return s;
}

}  // namespace ogs
}  // namespace ssos_eclss
