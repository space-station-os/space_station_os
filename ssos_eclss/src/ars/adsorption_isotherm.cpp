#include "ssos_eclss/ars/adsorption_isotherm.hpp"

#include <algorithm>
#include <cmath>

#include "ssos_eclss/common/units.hpp"

namespace ssos_eclss
{
namespace ars
{

// ===================== TothIsotherm =====================

TothIsotherm::TothIsotherm(const TothParams & params) : params_(params) {}

double TothIsotherm::q_max(double temperature_k) const
{
  return params_.q_m0 * std::exp(params_.chi * (1.0 - temperature_k / params_.T_ref));
}

double TothIsotherm::affinity(double temperature_k) const
{
  return params_.b0 *
         std::exp(params_.dH / (units::R_GAS * params_.T_ref) *
                  (params_.T_ref / temperature_k - 1.0));
}

double TothIsotherm::exponent(double temperature_k) const
{
  const double t = params_.t0 + params_.alpha * (1.0 - params_.T_ref / temperature_k);
  return std::clamp(t, 1.0e-3, 1.0);
}

double TothIsotherm::loading(double partial_pressure_pa, double temperature_k) const
{
  if (partial_pressure_pa <= 0.0) {
    return 0.0;
  }
  const double qm = q_max(temperature_k);
  const double b = affinity(temperature_k);
  const double t = exponent(temperature_k);
  const double bp = b * partial_pressure_pa;
  const double denom = std::pow(1.0 + std::pow(bp, t), 1.0 / t);
  return qm * bp / denom;
}

double TothIsotherm::dloading_dp(double partial_pressure_pa, double temperature_k) const
{
  if (partial_pressure_pa <= 0.0) {
    // Henry-law limit dq/dP -> qm * b.
    return q_max(temperature_k) * affinity(temperature_k);
  }
  const double qm = q_max(temperature_k);
  const double b = affinity(temperature_k);
  const double t = exponent(temperature_k);
  const double bp = b * partial_pressure_pa;
  const double bpt = std::pow(bp, t);
  const double base = 1.0 + bpt;
  // q = qm*b*P / base^(1/t);  dq/dP = qm*b / base^(1/t) * [1 - bpt/base]
  const double factor = std::pow(base, 1.0 / t);
  return qm * b / factor * (1.0 - bpt / base);
}

double TothIsotherm::dloading_dt(double partial_pressure_pa, double temperature_k) const
{
  const double h = 0.05;  // K
  const double q_plus = loading(partial_pressure_pa, temperature_k + h);
  const double q_minus = loading(partial_pressure_pa, temperature_k - h);
  return (q_plus - q_minus) / (2.0 * h);
}

// ===================== Competitive loading =====================

CompetitiveLoading competitive_loading(const TothIsotherm & co2,
                                       const TothIsotherm & h2o,
                                       double pp_co2_pa, double pp_h2o_pa,
                                       double temperature_k)
{
  CompetitiveLoading out{0.0, 0.0};

  // Single-component loadings as the uncoupled reference.
  const double q_co2_single = co2.loading(pp_co2_pa, temperature_k);
  const double q_h2o_single = h2o.loading(pp_h2o_pa, temperature_k);

  // Extended-Toth style coupling: each adsorbate's loading is scaled by a
  // competition factor built from the pseudo-Langmuir fractional occupancies.
  // Water dominates the shared sites, strongly suppressing CO2 capacity.
  const double b_co2 = co2.affinity(temperature_k);
  const double b_h2o = h2o.affinity(temperature_k);
  const double theta_co2 = b_co2 * std::max(pp_co2_pa, 0.0);
  const double theta_h2o = b_h2o * std::max(pp_h2o_pa, 0.0);
  const double sum = 1.0 + theta_co2 + theta_h2o;

  // CO2 sees suppression proportional to water occupancy.
  out.q_co2 = q_co2_single * (1.0 + theta_co2) / sum;
  // Water is the stronger adsorbate and is only weakly affected by CO2.
  out.q_h2o = q_h2o_single * (1.0 + theta_h2o) / (1.0 + theta_h2o + 0.05 * theta_co2);

  return out;
}

// ===================== Parameter factories =====================
// Defaults tuned to the 4BCO2 EDU (ICES-2021-313) and the Grace 544 / RK38
// material data in Knox & Cmarik (2019). Heats of adsorption and saturation
// loadings follow published zeolite 13X and silica-gel values.

TothParams default_co2_on_13x()
{
  TothParams p;
  p.q_m0 = 4.30;       // mol/kg
  p.chi = 2.20;        // -
  p.b0 = 9.0e-5;       // 1/Pa
  p.dH = 38000.0;      // J/mol
  p.t0 = 0.42;         // -
  p.alpha = 0.18;      // -
  p.T_ref = 298.15;    // K
  return p;
}

TothParams default_h2o_on_13x()
{
  TothParams p;
  p.q_m0 = 17.0;       // mol/kg
  p.chi = 1.10;
  p.b0 = 1.2e-3;       // 1/Pa (very strong affinity)
  p.dH = 52000.0;      // J/mol
  p.t0 = 0.50;
  p.alpha = 0.10;
  p.T_ref = 298.15;
  return p;
}

TothParams default_h2o_on_silica()
{
  TothParams p;
  p.q_m0 = 11.0;       // mol/kg
  p.chi = 0.90;
  p.b0 = 2.5e-4;       // 1/Pa
  p.dH = 45000.0;      // J/mol
  p.t0 = 0.60;
  p.alpha = 0.08;
  p.T_ref = 298.15;
  return p;
}

LDFParams default_desiccant_ldf()
{
  return LDFParams{8.0e-3, 2.0e-3};  // CO2 (unused), H2O [1/s]
}

LDFParams default_adsorbent_ldf()
{
  return LDFParams{1.5e-2, 3.0e-3};  // CO2, H2O [1/s]
}

BedGeometry default_desiccant_geometry()
{
  BedGeometry g;
  g.length = 0.165;            // m
  g.diameter = 0.1778;         // m (7 in)
  g.voidage = 0.36;            // -
  g.particle_diameter = 2.0e-3;  // m
  g.sorbent_density = 1200.0;  // kg/m^3
  g.n_cells = 50;
  return g;
}

BedGeometry default_adsorbent_geometry()
{
  BedGeometry g;
  g.length = 0.2667;           // m
  g.diameter = 0.1778;         // m (7 in)
  g.voidage = 0.36;            // -
  g.particle_diameter = 2.0e-3;  // m
  g.sorbent_density = 1100.0;  // kg/m^3
  g.n_cells = 50;
  return g;
}

BedThermal default_desiccant_thermal()
{
  BedThermal t;
  t.sorbent_cp = 920.0;
  t.wall_htc = 5.0;
  t.wall_area_per_vol = 4.0 / 0.1778;  // ~ 4/diameter for a cylinder
  t.wall_temp = 295.0;
  t.axial_thermal_cond = 0.5;
  t.gas_thermal_cond = 0.026;
  return t;
}

BedThermal default_adsorbent_thermal()
{
  BedThermal t = default_desiccant_thermal();
  t.sorbent_cp = 920.0;
  return t;
}

HeaterParams default_heater()
{
  HeaterParams h;
  h.total_power_w = 700.0;            // all 19 elements
  h.central_power_w = 700.0 * 7.0 / 19.0;  // central 7 elements during air-save
  h.max_temp_k = units::fahrenheit_to_kelvin(400.0);  // ~477.6 K
  return h;
}

CycleTiming default_cycle_timing()
{
  CycleTiming c;
  c.air_save_s = 10.0 * units::SECONDS_PER_MINUTE;  // 600 s
  c.adsorb_s = 60.0 * units::SECONDS_PER_MINUTE;    // 3600 s
  c.vacuum_s = 10.0 * units::SECONDS_PER_MINUTE;    // 600 s
  return c;
}

OperatingConditions default_operating_conditions()
{
  OperatingConditions o;
  o.inlet_flow_scfm = 26.0;
  o.inlet_ppco2_torr = 2.0;
  o.inlet_dewpoint_k = units::celsius_to_kelvin(4.0);
  o.cabin_temp_k = units::celsius_to_kelvin(22.0);
  o.cabin_pressure_pa = units::torr_to_pa(760.0);
  o.ltl_inlet_temp_k = units::celsius_to_kelvin(7.0);
  o.ltl_flow_gpm = 0.42;
  o.vacuum_pressure_pa = units::torr_to_pa(2.0);
  return o;
}

SystemEfficiency default_system_efficiency()
{
  SystemEfficiency e;
  e.capture_efficiency = 0.84;  // net/gross (paper 0.82-0.84)
  e.holdup_loss = 0.10;         // 8-12% cabin air lost to vacuum
  return e;
}

ArsParameters default_ars_parameters()
{
  ArsParameters p;
  p.desiccant_bed = default_desiccant_geometry();
  p.adsorbent_bed = default_adsorbent_geometry();
  p.desiccant_thermal = default_desiccant_thermal();
  p.adsorbent_thermal = default_adsorbent_thermal();
  p.co2_on_13x = default_co2_on_13x();
  p.h2o_on_13x = default_h2o_on_13x();
  p.h2o_on_silica = default_h2o_on_silica();
  p.desiccant_ldf = default_desiccant_ldf();
  p.adsorbent_ldf = default_adsorbent_ldf();
  p.heater = default_heater();
  p.cycle = default_cycle_timing();
  p.operating = default_operating_conditions();
  p.efficiency = default_system_efficiency();
  return p;
}

}  // namespace ars
}  // namespace ssos_eclss
