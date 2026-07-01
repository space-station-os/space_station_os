#include "ssos_eclss/sabatier/sabatier_system.hpp"

#include <algorithm>

#include "ssos_eclss/common/units.hpp"

namespace ssos_eclss
{
namespace sabatier
{

SabatierSystem::SabatierSystem(const SabatierParameters & params) : params_(params)
{
  kinetics_ = std::make_unique<ReactorKinetics>(params_.kinetics);
  reactor_temp_k_ = params_.reactor.operating_temp_k;
}

void SabatierSystem::reset(double temperature_k) { reactor_temp_k_ = temperature_k; }

void SabatierSystem::set_parameters(const SabatierParameters & params)
{
  params_ = params;
  kinetics_->set_params(params_.kinetics);
}

SabatierResult SabatierSystem::step(double dt, double co2_in_mol_s, double h2_in_mol_s)
{
  SabatierResult r{};
  co2_in_mol_s = std::max(0.0, co2_in_mol_s);
  h2_in_mol_s = std::max(0.0, h2_in_mol_s);

  // Stoichiometry: 4 H2 per CO2. Determine the limiting reactant feed.
  const double co2_equiv_from_h2 = h2_in_mol_s / 4.0;
  double co2_reactable = co2_in_mol_s;
  if (co2_equiv_from_h2 < co2_in_mol_s) {
    co2_reactable = co2_equiv_from_h2;
    r.hydrogen_limited = true;
  }

  // Kinetic / equilibrium conversion at the current reactor temperature.
  const double x = kinetics_->conversion(reactor_temp_k_);
  r.conversion = x;

  r.co2_consumed_mol_s = co2_reactable * x;
  r.h2_consumed_mol_s = 4.0 * r.co2_consumed_mol_s;
  r.ch4_produced_mol_s = r.co2_consumed_mol_s;          // 1 CH4 per CO2
  r.water_produced_mol_s = 2.0 * r.co2_consumed_mol_s;  // 2 H2O per CO2
  r.water_produced_kg_s = r.water_produced_mol_s * units::M_H2O;

  // Exothermic heat release.
  r.reaction_heat_w = r.co2_consumed_mol_s * params_.kinetics.heat_of_reaction;

  // Lumped thermal update with thermostatic trim heater:
  //   dT/dt = (Q_rxn + Q_heater - UA(T - Tamb)) / C
  // Heater covers the loss/exotherm deficit plus a proportional term toward the
  // setpoint (clamped >= 0), holding conversion at low throughput.
  const double q_loss = params_.reactor.heat_loss_coeff_w_k *
                        (reactor_temp_k_ - params_.reactor.ambient_temp_k);
  const double setpoint = params_.reactor.operating_temp_k;
  double q_heater = (q_loss - r.reaction_heat_w) +
                    params_.reactor.heater_gain_w_k * (setpoint - reactor_temp_k_);
  q_heater = std::clamp(q_heater, 0.0, params_.reactor.trim_heater_max_w);
  r.heater_power_w = q_heater;
  const double dT = (r.reaction_heat_w + q_heater - q_loss) /
                    std::max(params_.reactor.thermal_mass_j_k, 1.0);
  reactor_temp_k_ += dT * dt;
  r.reactor_temp_k = reactor_temp_k_;
  return r;
}

// ---- Parameter factories ----

KineticsParams default_kinetics_params()
{
  // Calibrated to the ISS Sabatier reactor behaviour (ICES-2018-155): kinetics
  // become fast enough to reach equilibrium above ~375 C (648 K), and the
  // exothermic equilibrium conversion declines at higher temperature. At the
  // ~648 K operating point the achievable conversion is ~0.95, matching the
  // ISS reactor's ~95% (90% hot front + 5% cooled section).
  KineticsParams p{};
  p.pre_exponential = 7.67e5;       // 1/s
  p.activation_energy = 7.0e4;      // J/mol (steep kinetic onset near 375 C)
  p.max_conversion = 0.99;          // equilibrium ceiling at low temperature
  p.residence_time_s = 2.0;         // s
  p.heat_of_reaction = 165400.0;    // J/mol CO2 (dH = -165.4 kJ/mol)
  p.equilibrium_knee_temp_k = 600.0;       // equilibrium declines above ~327 C
  p.equilibrium_decline_per_k = 8.0e-4;    // conversion lost per K above knee
  return p;
}

ReactorParams default_reactor_params()
{
  ReactorParams p{};
  p.operating_temp_k = 648.0;     // ~375 C — kinetic/thermodynamic sweet spot
  p.thermal_mass_j_k = 2000.0;
  p.heat_loss_coeff_w_k = 0.5;
  p.ambient_temp_k = 295.0;
  p.trim_heater_max_w = 600.0;   // sized with margin over the ~25 W steady deficit
  p.heater_gain_w_k = 50.0;
  return p;
}

SabatierParameters default_sabatier_parameters()
{
  SabatierParameters p{};
  p.kinetics = default_kinetics_params();
  p.reactor = default_reactor_params();
  return p;
}

}  // namespace sabatier
}  // namespace ssos_eclss
