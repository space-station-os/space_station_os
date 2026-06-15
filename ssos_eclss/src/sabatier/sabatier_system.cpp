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

  // Lumped reactor thermal update: dT/dt = (Q_rxn - UA(T - Tamb)) / C.
  const double q_loss = params_.reactor.heat_loss_coeff_w_k *
                        (reactor_temp_k_ - params_.reactor.ambient_temp_k);
  const double dT = (r.reaction_heat_w - q_loss) /
                    std::max(params_.reactor.thermal_mass_j_k, 1.0);
  reactor_temp_k_ += dT * dt;
  r.reactor_temp_k = reactor_temp_k_;
  return r;
}

// ---- Parameter factories ----

KineticsParams default_kinetics_params()
{
  KineticsParams p{};
  p.pre_exponential = 5.0e3;      // 1/s
  p.activation_energy = 4.0e4;    // J/mol
  p.max_conversion = 0.95;        // equilibrium-limited
  p.residence_time_s = 2.0;       // s
  p.heat_of_reaction = 165000.0;  // J/mol CO2 (exothermic)
  return p;
}

ReactorParams default_reactor_params()
{
  ReactorParams p{};
  p.operating_temp_k = 600.0;     // ~327 C
  p.thermal_mass_j_k = 2000.0;
  // Well-insulated small reactor: the exotherm (~150 W at flight CO2 rates)
  // sustains the operating temperature against this loss.
  p.heat_loss_coeff_w_k = 0.5;
  p.ambient_temp_k = 295.0;
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
