#include "ssos_eclss/cabin/leak_model.hpp"

#include <cmath>

#include "ssos_eclss/common/units.hpp"

namespace ssos_eclss
{
namespace cabin
{

LeakModel::LeakModel(const LeakParams & params) : params_(params) {}

double LeakModel::mass_leak_rate(const CabinAtmosphere & atm) const
{
  const double p0 = atm.total_pressure_pa();
  const double t0 = atm.temperature_k();
  if (p0 <= 0.0 || t0 <= 0.0) {
    return 0.0;
  }
  // Mixture molar mass from composition.
  const double m_mix =
      atm.mole_fraction(Gas::O2) * units::M_O2 +
      atm.mole_fraction(Gas::CO2) * units::M_CO2 +
      atm.mole_fraction(Gas::N2) * units::M_N2 +
      atm.mole_fraction(Gas::H2O) * units::M_H2O;
  const double r_specific = units::R_GAS / std::max(m_mix, 1.0e-6);

  // Choked (sonic) orifice flow to vacuum, gamma ~ 1.4:
  //   m_dot = Cd * A * sqrt(gamma * rho0 * P0) * (2/(gamma+1))^((gamma+1)/(2(gamma-1)))
  const double gamma = 1.4;
  const double rho0 = p0 / (r_specific * t0);
  const double choke =
      std::pow(2.0 / (gamma + 1.0), (gamma + 1.0) / (2.0 * (gamma - 1.0)));
  return params_.discharge_coeff * effective_area() * std::sqrt(gamma * rho0 * p0) *
         choke;
}

GasFlows LeakModel::leak_flows(const CabinAtmosphere & atm) const
{
  const double m_dot = mass_leak_rate(atm);  // kg/s
  GasFlows f{};
  if (m_dot <= 0.0) {
    return f;
  }
  // Distribute by mole fraction: outflow is well-mixed gas.
  const double m_mix =
      atm.mole_fraction(Gas::O2) * units::M_O2 +
      atm.mole_fraction(Gas::CO2) * units::M_CO2 +
      atm.mole_fraction(Gas::N2) * units::M_N2 +
      atm.mole_fraction(Gas::H2O) * units::M_H2O;
  const double n_dot_total = m_dot / std::max(m_mix, 1.0e-6);  // mol/s

  f.o2 = -atm.mole_fraction(Gas::O2) * n_dot_total;
  f.co2 = -atm.mole_fraction(Gas::CO2) * n_dot_total;
  f.n2 = -atm.mole_fraction(Gas::N2) * n_dot_total;
  f.h2o = -atm.mole_fraction(Gas::H2O) * n_dot_total;
  return f;
}

}  // namespace cabin
}  // namespace ssos_eclss
