#include "ssos_eclss/ogs/oxygen_generator_system.hpp"

#include <algorithm>

#include "ssos_eclss/common/units.hpp"

namespace ssos_eclss
{
namespace ogs
{

OxygenGeneratorSystem::OxygenGeneratorSystem(const OgsParameters & params)
: params_(params)
{
  stack_ = std::make_unique<ElectrolysisStackModel>(params_.stack, params_.cell);
  separator_ = std::make_unique<GasSeparatorModel>(params_.separator);
  reset(params_.operating.feedwater_temp_k > 0.0 ? 330.0 : 330.0);
}

void OxygenGeneratorSystem::reset(double temperature_k)
{
  stack_->reset(temperature_k);
}

void OxygenGeneratorSystem::set_parameters(const OgsParameters & params)
{
  params_ = params;
  stack_->set_params(params_.stack);
  stack_->set_cell_params(params_.cell);
  separator_->set_params(params_.separator);
}

OgsResult OxygenGeneratorSystem::step(double dt, double current_a,
                                      double available_water_mol_s)
{
  OgsResult r{};

  // Water demanded by electrolysis at this current: H2O = 2 * O2 = I*n/(2F).
  const double n_cells = static_cast<double>(std::max(params_.stack.n_cells, 1));
  const double water_demand = current_a * n_cells / (2.0 * units::FARADAY);

  // Limit current if feed water is insufficient.
  double effective_current = current_a;
  if (available_water_mol_s < water_demand && water_demand > 0.0) {
    effective_current = current_a * available_water_mol_s / water_demand;
    r.feedwater_limited = true;
  }

  const StackState s = stack_->step(dt, effective_current, params_.operating.cell_pressure_pa,
                                    params_.operating.coolant_temp_k);

  // Separate carried water from the O2 product.
  const SeparationResult sep = separator_->separate(s.o2_production_mol_s, 0.0);

  r.o2_production_mol_s = sep.dry_gas_mol_s;
  r.h2_production_mol_s = s.h2_production_mol_s;
  r.water_consumed_mol_s = s.o2_production_mol_s * 2.0;  // 2 H2O per O2
  r.o2_production_kg_day =
      r.o2_production_mol_s * units::M_O2 * units::SECONDS_PER_DAY;
  r.water_consumed_kg_day =
      r.water_consumed_mol_s * units::M_H2O * units::SECONDS_PER_DAY;
  r.stack_voltage = s.total_voltage;
  r.stack_power_w = s.power_w;
  r.stack_temperature_k = s.temperature_k;

  const double o2_kg_s = r.o2_production_mol_s * units::M_O2;
  r.specific_energy_wh_kg =
      (o2_kg_s > 0.0) ? s.power_w / (o2_kg_s * units::SECONDS_PER_HOUR) : 0.0;
  return r;
}

OgsResult OxygenGeneratorSystem::step_nominal(double dt)
{
  return step(dt, params_.operating.stack_current_a, 1.0e9);
}

// ---- Parameter factories (ISS OGA-class defaults) ----

CellParams default_cell_params()
{
  // Calibrated to the ISS OGA / AOGA (ICES-2023-311): a 28-cell stack delivers
  // 9.25 kg O2/day at 46.9 A with a nominal cell voltage of ~1.7 V. The active
  // area / resistance / exchange-current values reproduce ~1.7 V at the 46.9 A
  // operating point, and the Faradaic efficiency (0.983) matches the measured
  // 9.25 kg/day vs the 9.41 kg/day Faradaic ideal.
  CellParams c{};
  c.active_area_m2 = 0.023;          // ~230 cm^2 (=> ~0.20 A/cm^2 at 46.9 A)
  c.exchange_current_density = 1.0;  // A/m^2
  c.limiting_current_density = 2.0e4;   // A/m^2
  c.membrane_resistance = 1.3e-4;    // ohm*m^2
  c.charge_transfer_coeff = 0.5;
  c.reference_temp_k = 298.15;
  c.faradaic_efficiency = 0.983;
  return c;
}

StackParams default_stack_params()
{
  StackParams s{};
  s.n_cells = 28;                    // ISS OGA / flight AOGA 28-cell stack
  s.thermal_mass_j_k = 5.0e4;
  s.heat_loss_coeff_w_k = 40.0;
  return s;
}

SeparatorParams default_separator_params()
{
  SeparatorParams s{};
  s.efficiency = 0.98;
  s.carryover_fraction = 0.01;
  return s;
}

OgsOperating default_ogs_operating()
{
  // ISS OGA PSM current is selectable 10-46.9 A (Process). 46.9 A => 9.25 kg/day
  // (supports 10.88 crew at 0.85 kg O2/day). Default to a ~6-crew nominal.
  OgsOperating o{};
  o.stack_current_a = 27.0;          // ~5.3 kg O2/day (28 cells)
  o.feedwater_temp_k = units::celsius_to_kelvin(25.0);
  o.coolant_temp_k = units::celsius_to_kelvin(18.0);
  o.cell_pressure_pa = 165474.0;     // ~24 psia recirculation-loop pressure
  return o;
}

OgsParameters default_ogs_parameters()
{
  OgsParameters p{};
  p.cell = default_cell_params();
  p.stack = default_stack_params();
  p.separator = default_separator_params();
  p.operating = default_ogs_operating();
  return p;
}

}  // namespace ogs
}  // namespace ssos_eclss
