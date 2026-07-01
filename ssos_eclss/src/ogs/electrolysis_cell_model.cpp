#include "ssos_eclss/ogs/electrolysis_cell_model.hpp"

#include <algorithm>
#include <cmath>

#include "ssos_eclss/common/units.hpp"

namespace ssos_eclss
{
namespace ogs
{

namespace
{
constexpr double kE0 = 1.229;          // standard reversible voltage at 298 K [V]
constexpr double kThermoneutral = 1.481;  // thermoneutral voltage [V]
constexpr int kElectrons = 2;          // electrons per H2O for the H2 half-cell
}  // namespace

ElectrolysisCellModel::ElectrolysisCellModel(const CellParams & params)
: params_(params)
{}

double ElectrolysisCellModel::reversible_voltage(double temperature_k,
                                                 double pressure_pa) const
{
  // Temperature correction of E0 (dE/dT ~ -0.9 mV/K for water splitting).
  const double e_temp = kE0 - 0.0009 * (temperature_k - 298.15);
  // Nernst pressure term: E = E_T + (R T / nF) ln(pH2 * sqrt(pO2) / aH2O).
  // Product gases at operating pressure (atm-referenced), liquid water aH2O~1.
  const double p_atm = std::max(pressure_pa / units::STD_PRESSURE_PA, 1.0e-6);
  const double nernst = units::R_GAS * temperature_k / (kElectrons * units::FARADAY) *
                        std::log(p_atm * std::sqrt(p_atm));
  return e_temp + nernst;
}

CellState ElectrolysisCellModel::solve(double current_a, double temperature_k,
                                       double pressure_pa) const
{
  CellState s{};
  const double i = std::max(current_a, 0.0) / params_.active_area_m2;  // A/m^2

  s.reversible_voltage = reversible_voltage(temperature_k, pressure_pa);

  // Activation overpotential (Butler-Volmer high-field / Tafel form).
  const double rt_anf = units::R_GAS * temperature_k /
                        (params_.charge_transfer_coeff * kElectrons * units::FARADAY);
  s.activation_overpotential =
      (i > 0.0) ? rt_anf * std::asinh(i / (2.0 * params_.exchange_current_density))
                : 0.0;

  // Ohmic overpotential through the membrane (area-specific resistance).
  s.ohmic_overpotential = i * params_.membrane_resistance;

  // Concentration overpotential near the limiting current.
  const double ratio = std::clamp(i / params_.limiting_current_density, 0.0, 0.999);
  s.concentration_overpotential =
      (i > 0.0) ? -units::R_GAS * temperature_k / (kElectrons * units::FARADAY) *
                      std::log(1.0 - ratio)
                : 0.0;

  s.voltage = s.reversible_voltage + s.activation_overpotential +
              s.ohmic_overpotential + s.concentration_overpotential;

  // Faraday's law: per cell H2 = I/(2F), O2 = I/(4F), scaled by the Faradaic
  // (current) efficiency to match measured production.
  const double fe = params_.faradaic_efficiency;
  s.h2_production_mol_s = fe * current_a / (2.0 * units::FARADAY);
  s.o2_production_mol_s = fe * current_a / (4.0 * units::FARADAY);

  s.power_w = s.voltage * current_a;
  s.efficiency = (s.voltage > 0.0) ? kThermoneutral / s.voltage : 0.0;
  return s;
}

}  // namespace ogs
}  // namespace ssos_eclss
