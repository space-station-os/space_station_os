#include "ssos_eclss/ars/four_bed_system.hpp"

#include <algorithm>
#include <cmath>

#include "ssos_eclss/common/fluid_dynamics.hpp"
#include "ssos_eclss/common/gas_properties.hpp"
#include "ssos_eclss/common/thermodynamics.hpp"
#include "ssos_eclss/common/units.hpp"

namespace ssos_eclss
{
namespace ars
{

FourBedSystem::FourBedSystem(const ArsParameters & params)
: params_(params),
  co2_iso_(params.co2_on_13x),
  h2o_iso_(params.h2o_on_13x),
  h2o_silica_iso_(params.h2o_on_silica)
{
  reconstruct();
}

void FourBedSystem::reconstruct()
{
  co2_iso_ = TothIsotherm(params_.co2_on_13x);
  h2o_iso_ = TothIsotherm(params_.h2o_on_13x);
  h2o_silica_iso_ = TothIsotherm(params_.h2o_on_silica);

  // A zero-capacity CO2 isotherm for desiccant beds (water only).
  TothParams no_co2 = params_.co2_on_13x;
  no_co2.q_m0 = 0.0;
  TothIsotherm desiccant_co2(no_co2);

  for (int i = 0; i < 2; ++i) {
    desiccant_[i] = std::make_unique<BedModel>(
      params_.desiccant_bed, params_.desiccant_thermal, desiccant_co2,
      h2o_silica_iso_, params_.desiccant_ldf, /*is_desiccant=*/true);
    adsorbent_[i] = std::make_unique<BedModel>(
      params_.adsorbent_bed, params_.adsorbent_thermal, co2_iso_, h2o_iso_,
      params_.adsorbent_ldf, /*is_desiccant=*/false);
  }

  // Precooler sized to the design air/coolant flows.
  PrecoolerParams pc{};
  const double rho_air = gas::gas_density(params_.operating.cabin_pressure_pa,
                                          params_.operating.cabin_temp_k, units::M_AIR);
  const double q_air = units::scfm_to_m3s(params_.operating.inlet_flow_scfm);
  pc.air_mass_flow = rho_air * q_air;
  pc.coolant_mass_flow = 1000.0 * units::gpm_to_m3s(params_.operating.ltl_flow_gpm);
  pc.air_cp = thermo::cp_mass(thermo::Species::AIR, params_.operating.cabin_temp_k);
  pc.coolant_cp = thermo::cp_liquid_water(params_.operating.ltl_inlet_temp_k);
  // Size UA for a high-effectiveness exchanger (exit within a few K of coolant).
  pc.ua = 3.0 * std::min(pc.air_mass_flow * pc.air_cp,
                         pc.coolant_mass_flow * pc.coolant_cp);
  precooler_ = std::make_unique<PrecoolerModel>(pc);

  // Blower curve targeting ~26 SCFM at ~37-40 in-H2O.
  BlowerParams bp{};
  bp.rated_rpm = 12000.0;
  bp.shutoff_dp = 2.0 * units::in_h2o_to_pa(40.0);
  bp.max_rpm = 18000.0;
  // Choose curve so design flow yields design head.
  bp.curve_quad = units::in_h2o_to_pa(40.0) / (q_air * q_air);
  blower_ = std::make_unique<BlowerModel>(bp);

  // Air-save scroll pump.
  AirSavePumpParams asp{};
  asp.void_volume_m3 = params_.adsorbent_bed.volume() * params_.adsorbent_bed.voidage;
  asp.displacement_m3s = asp.void_volume_m3 / 120.0;  // empties void in ~2 min
  asp.ultimate_pressure_pa = units::torr_to_pa(20.0);
  air_save_pump_ = std::make_unique<AirSavePumpModel>(asp);

  // Selector / repressurisation valve (0->800 torr in ~15 s).
  ValveParams vp{};
  vp.cv = 1.0e-4;
  vp.stroke_time_s = 2.0;
  vp.repress_time_s = 15.0 / 3.0;  // ~3 time constants to fill
  valve_ = std::make_unique<ValveModel>(vp);

  cycle_ = std::make_unique<CycleStateMachine>(params_.cycle, params_.heater);

  // System resistance from Ergun across the desiccant + adsorbent beds at the
  // design velocity: dP = R * Q^2.
  const double area = params_.adsorbent_bed.cross_area();
  const double v = fluid::superficial_velocity(q_air, area);
  const double mu = gas::sutherland_viscosity(params_.operating.cabin_temp_k);
  const double dp = fluid::ergun_pressure_drop(v, params_.adsorbent_bed.voidage,
                                               params_.adsorbent_bed.particle_diameter,
                                               rho_air, mu,
                                               params_.adsorbent_bed.length +
                                               params_.desiccant_bed.length);
  system_resistance_ = (q_air > 0.0) ? dp / (q_air * q_air) : 0.0;

  reset(params_.operating.cabin_temp_k);
}

void FourBedSystem::reset(double temperature_k)
{
  for (int i = 0; i < 2; ++i) {
    desiccant_[i]->reset(temperature_k);
    adsorbent_[i]->reset(temperature_k);
  }
  if (cycle_) {
    cycle_->reset();
  }
}

void FourBedSystem::set_parameters(const ArsParameters & params)
{
  // Preserve geometry/isotherm fields that require a rebuild; apply the rest.
  const BedGeometry des_geom = params_.desiccant_bed;
  const BedGeometry ads_geom = params_.adsorbent_bed;
  params_ = params;
  params_.desiccant_bed = des_geom;
  params_.adsorbent_bed = ads_geom;
  // Tunable (non-geometry) updates that don't require a rebuild: cycle, heater,
  // operating set-points and the net-capture efficiency are used directly.
  if (cycle_) {
    cycle_->set_timing(params_.cycle);
    cycle_->set_heater(params_.heater);
  }
}

double FourBedSystem::inlet_co2_mass_rate(const CabinConditions & cabin,
                                          double flow_m3s) const
{
  // Molar flow of CO2 = (ppCO2/Ptot) * (Ptot*Q)/(R*T) = ppCO2 * Q / (R*T).
  const double n_co2 = cabin.co2_partial_pressure_pa * flow_m3s /
                       (units::R_GAS * cabin.temperature_k);
  return n_co2 * units::M_CO2;  // kg/s
}

double FourBedSystem::design_co2_removal_kg_day() const
{
  CabinConditions cabin{};
  cabin.co2_partial_pressure_pa = units::torr_to_pa(params_.operating.inlet_ppco2_torr);
  cabin.temperature_k = params_.operating.cabin_temp_k;
  cabin.total_pressure_pa = params_.operating.cabin_pressure_pa;
  const double q_air = units::scfm_to_m3s(params_.operating.inlet_flow_scfm);
  const double gross = inlet_co2_mass_rate(cabin, q_air);
  return gross * params_.efficiency.capture_efficiency * units::SECONDS_PER_DAY;
}

ArsResult FourBedSystem::step(double dt, const CabinConditions & cabin)
{
  ArsResult res{};

  // ---- Blower / flow ----
  const BlowerResult blower = blower_->solve(
    system_resistance_, BlowerControl::CONSTANT_FLOW,
    units::scfm_to_m3s(params_.operating.inlet_flow_scfm));
  const double q_air = blower.flow_m3s;
  const double area = params_.adsorbent_bed.cross_area();
  const double v_super = fluid::superficial_velocity(q_air, area);

  // ---- Precooler ----
  const PrecoolerResult pc = precooler_->solve(cabin.temperature_k,
                                               params_.operating.ltl_inlet_temp_k);

  // ---- Cycle commands ----
  const int ads_train = cycle_->adsorbing_train();
  const int regen_train = cycle_->regenerating_train();
  const TrainCommand ads_cmd = cycle_->command_for_train(ads_train);
  const TrainCommand regen_cmd = cycle_->command_for_train(regen_train);

  // ---- Adsorbing train: cabin -> desiccant -> adsorbent ----
  BedInlet des_in{};
  des_in.velocity_superficial = v_super;
  des_in.c_co2 = gas::concentration_from_pp(cabin.co2_partial_pressure_pa, pc.air_outlet_temp);
  des_in.c_h2o = gas::concentration_from_pp(cabin.h2o_partial_pressure_pa, pc.air_outlet_temp);
  des_in.temperature_k = pc.air_outlet_temp;
  des_in.pressure_pa = cabin.total_pressure_pa;

  BedOutputs des_out = desiccant_[ads_train]->step(dt, des_in, ads_cmd.desiccant_mode,
                                                   ads_cmd.desiccant_heater_w);

  BedInlet ads_in{};
  ads_in.velocity_superficial = v_super;
  ads_in.c_co2 = des_out.outlet_c_co2;
  ads_in.c_h2o = des_out.outlet_c_h2o;
  ads_in.temperature_k = des_out.outlet_temperature;
  ads_in.pressure_pa = cabin.total_pressure_pa;

  BedOutputs ads_out = adsorbent_[ads_train]->step(dt, ads_in, ads_cmd.adsorbent_mode,
                                                   ads_cmd.adsorbent_heater_w);

  // ---- Regenerating train: desorb under vacuum with heat ----
  BedInlet regen_in{};
  regen_in.velocity_superficial = 0.0;
  regen_in.c_co2 = 0.0;
  regen_in.c_h2o = 0.0;
  regen_in.temperature_k = params_.operating.cabin_temp_k;
  regen_in.pressure_pa = params_.operating.vacuum_pressure_pa;

  const double regen_co2_before = adsorbent_[regen_train]->total_co2_loading_mol();
  adsorbent_[regen_train]->step(dt, regen_in, regen_cmd.adsorbent_mode,
                                regen_cmd.adsorbent_heater_w);
  desiccant_[regen_train]->step(dt, regen_in, regen_cmd.desiccant_mode,
                                regen_cmd.desiccant_heater_w);
  const double regen_co2_after = adsorbent_[regen_train]->total_co2_loading_mol();
  const double desorbed_mol = std::max(0.0, regen_co2_before - regen_co2_after);

  // ---- Air-save pump (during AIR_SAVE phase only) ----
  double air_saved_rate = 0.0;
  if (cycle_->regen_phase() == RegenPhase::AIR_SAVE) {
    const AirSaveResult as = air_save_pump_->pump(dt, cabin.total_pressure_pa,
                                                  params_.operating.cabin_temp_k);
    air_saved_rate = as.air_recovered_mol * units::M_AIR / dt;
  }

  // ---- Valve actuation toward the active configuration ----
  valve_->command(1.0);
  valve_->update(dt);

  // ---- Net CO2 removal (system, continuous) ----
  // For the cyclic two-train 4BMS, the continuous system removal is the
  // cycle-average: the inlet CO2 load times the net capture efficiency. A single
  // bed's instantaneous capture (ads_out.co2_capture_rate) legitimately swings
  // through adsorb -> breakthrough -> swap, but while one train approaches
  // breakthrough the other has just been regenerated, so the alternating trains
  // sustain throughput at this average. Reporting the single-bed instantaneous
  // value made the rate (and the health flag) oscillate every step.
  const double inlet_co2_kg_s = inlet_co2_mass_rate(cabin, q_air);
  res.co2_removal_rate_kg_s = inlet_co2_kg_s * params_.efficiency.capture_efficiency;
  res.co2_removal_rate_kg_day = res.co2_removal_rate_kg_s * units::SECONDS_PER_DAY;

  // ---- Advance cycle ----
  cycle_->update(dt);

  // ---- Fill out result ----
  res.scrubbed_co2_pp_pa = gas::partial_pressure(ads_out.outlet_c_co2,
                                                 ads_out.outlet_temperature);
  res.scrubbed_h2o_pp_pa = gas::partial_pressure(ads_out.outlet_c_h2o,
                                                 ads_out.outlet_temperature);
  res.return_air_temp_k = ads_out.outlet_temperature;
  res.system_pressure_drop_pa = des_out.pressure_drop_pa + ads_out.pressure_drop_pa;
  res.blower_flow_scfm = units::m3s_to_scfm(q_air);
  res.blower_power_w = blower.power_w;
  res.precooler_exit_temp_k = pc.air_outlet_temp;
  res.max_bed_temp_k = std::max({desiccant_[0]->max_solid_temperature(),
                                 desiccant_[1]->max_solid_temperature(),
                                 adsorbent_[0]->max_solid_temperature(),
                                 adsorbent_[1]->max_solid_temperature()});
  res.co2_desorbed_rate_kg_s = desorbed_mol * units::M_CO2 / dt;
  res.air_saved_rate_kg_s = air_saved_rate;
  res.adsorbing_train = ads_train;
  return res;
}

}  // namespace ars
}  // namespace ssos_eclss
