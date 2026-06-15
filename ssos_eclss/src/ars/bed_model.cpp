#include "ssos_eclss/ars/bed_model.hpp"

#include <algorithm>
#include <cmath>

#include "ssos_eclss/common/fluid_dynamics.hpp"
#include "ssos_eclss/common/gas_properties.hpp"
#include "ssos_eclss/common/heat_transfer.hpp"
#include "ssos_eclss/common/numerical/cfl_control.hpp"
#include "ssos_eclss/common/units.hpp"

namespace ssos_eclss
{
namespace ars
{

namespace
{
constexpr double kGasCp = 1010.0;        // process-gas cp [J/(kg*K)] (~air)
constexpr double kAxialDispersion = 1.0e-4;  // mass axial dispersion [m^2/s]
constexpr double kHeaterMaxTempK = 533.0;  // heater cutoff (~500 F) [K]
constexpr std::size_t kMaxSubsteps = 4000;
}  // namespace

BedModel::BedModel(const BedGeometry & geom, const BedThermal & thermal,
                   const TothIsotherm & co2, const TothIsotherm & h2o,
                   const LDFParams & ldf, bool is_desiccant)
: geom_(geom), thermal_(thermal), co2_iso_(co2), h2o_iso_(h2o), ldf_(ldf),
  is_desiccant_(is_desiccant)
{
  n_ = std::max<std::size_t>(geom_.n_cells, 1);
  dz_ = geom_.length / static_cast<double>(n_);
  rho_bulk_ = (1.0 - geom_.voidage) * geom_.sorbent_density;
  reset(295.0);
}

void BedModel::reset(double temperature_k)
{
  c_co2_.assign(n_, 0.0);
  c_h2o_.assign(n_, 0.0);
  q_co2_.assign(n_, 0.0);
  q_h2o_.assign(n_, 0.0);
  tg_.assign(n_, temperature_k);
  ts_.assign(n_, temperature_k);
}

void BedModel::equilibrate(double pp_co2_pa, double pp_h2o_pa, double temperature_k)
{
  for (std::size_t i = 0; i < n_; ++i) {
    tg_[i] = temperature_k;
    ts_[i] = temperature_k;
    c_co2_[i] = gas::concentration_from_pp(pp_co2_pa, temperature_k);
    c_h2o_[i] = gas::concentration_from_pp(pp_h2o_pa, temperature_k);
    const auto eq = competitive_loading(co2_iso_, h2o_iso_, pp_co2_pa, pp_h2o_pa,
                                        temperature_k);
    q_co2_[i] = is_desiccant_ ? 0.0 : eq.q_co2;
    q_h2o_[i] = eq.q_h2o;
  }
}

double BedModel::total_co2_loading_mol() const
{
  const double mass_per_cell = rho_bulk_ * geom_.cross_area() * dz_;
  double total = 0.0;
  for (double q : q_co2_) {
    total += q * mass_per_cell;
  }
  return total;
}

double BedModel::total_h2o_loading_mol() const
{
  const double mass_per_cell = rho_bulk_ * geom_.cross_area() * dz_;
  double total = 0.0;
  for (double q : q_h2o_) {
    total += q * mass_per_cell;
  }
  return total;
}

double BedModel::mean_solid_temperature() const
{
  double s = 0.0;
  for (double t : ts_) {
    s += t;
  }
  return s / static_cast<double>(n_);
}

double BedModel::max_solid_temperature() const
{
  return *std::max_element(ts_.begin(), ts_.end());
}

double BedModel::loading_fraction() const
{
  const double qm = is_desiccant_ ? h2o_iso_.params().q_m0 : co2_iso_.params().q_m0;
  if (qm <= 0.0) {
    return 0.0;
  }
  const std::vector<double> & q = is_desiccant_ ? q_h2o_ : q_co2_;
  double sum = 0.0;
  for (double v : q) {
    sum += v;
  }
  const double mean = sum / static_cast<double>(n_);
  return std::clamp(mean / qm, 0.0, 1.0);
}

double BedModel::bed_pressure_drop(const BedInlet & inlet) const
{
  const double rho = gas::gas_density(inlet.pressure_pa, inlet.temperature_k,
                                      units::M_AIR);
  const double mu = gas::sutherland_viscosity(inlet.temperature_k);
  return fluid::ergun_pressure_drop(std::abs(inlet.velocity_superficial),
                                    geom_.voidage, geom_.particle_diameter, rho, mu,
                                    geom_.length);
}

void BedModel::integrate_substep(double h, const BedInlet & inlet, BedMode mode,
                                 double heater_power_w)
{
  const double eps = geom_.voidage;
  const double v = inlet.velocity_superficial;  // superficial [m/s]
  const bool flowing = (mode == BedMode::ADSORBING || mode == BedMode::AIR_SAVE ||
                        mode == BedMode::REPRESSURIZING);
  const bool pumping = (mode == BedMode::DESORBING || mode == BedMode::VACUUM);

  // Gas / transport coefficients (evaluated once per substep at inlet state).
  const double rho_g = gas::gas_density(inlet.pressure_pa, inlet.temperature_k,
                                        units::M_AIR);
  const double mu = gas::sutherland_viscosity(inlet.temperature_k);
  const double re = fluid::reynolds_number(rho_g, v, geom_.particle_diameter, mu);
  const double pr = heat::prandtl_number(kGasCp, mu, thermal_.gas_thermal_cond);
  const double nu = heat::nusselt_wakao_kaguei(re, pr);
  const double h_gs = heat::gas_solid_htc(nu, thermal_.gas_thermal_cond,
                                          geom_.particle_diameter);
  const double a_v = heat::specific_surface_area(eps, geom_.particle_diameter);
  const double exch = h_gs * a_v;  // volumetric gas-solid exchange [W/(m^3*K)]
  const double u_wall = thermal_.wall_htc * thermal_.wall_area_per_vol;
  const double cap_g = std::max(eps * rho_g * kGasCp, 1.0e-9);
  const double cap_s = std::max((1.0 - eps) * geom_.sorbent_density *
                                thermal_.sorbent_cp, 1.0e-9);

  const double c_vac = pumping
      ? gas::concentration_from_pp(2.0 * units::TORR_TO_PA,
                                   std::max(ts_.front(), 200.0))
      : 0.0;

  // Snapshots of the old fields (advection/conduction use old neighbour values).
  const std::vector<double> c_co2_old = c_co2_;
  const std::vector<double> c_h2o_old = c_h2o_;
  const std::vector<double> tg_old = tg_;
  const std::vector<double> ts_old = ts_;

  for (std::size_t i = 0; i < n_; ++i) {
    // ---- Equilibrium loading + LDF sorption rate ----
    const double pp_co2 = gas::partial_pressure(std::max(c_co2_old[i], 0.0), ts_old[i]);
    const double pp_h2o = gas::partial_pressure(std::max(c_h2o_old[i], 0.0), ts_old[i]);
    const auto eq = competitive_loading(co2_iso_, h2o_iso_, pp_co2, pp_h2o, ts_old[i]);
    const double q_co2_star = is_desiccant_ ? 0.0 : eq.q_co2;
    const double q_h2o_star = eq.q_h2o;
    const double dqco2 = is_desiccant_ ? 0.0 : ldf_.k_co2 * (q_co2_star - q_co2_[i]);
    const double dqh2o = ldf_.k_h2o * (q_h2o_star - q_h2o_[i]);

    // ---- Gas-phase mass balance (explicit) ----
    auto upwind = [&](const std::vector<double> & c, double c_in) {
      if (!flowing || v <= 0.0) {
        return 0.0;
      }
      const double up = (i == 0) ? c_in : c[i - 1];
      return -v * (c[i] - up) / dz_;
    };
    auto diffuse = [&](const std::vector<double> & c) {
      const double left = (i == 0) ? c[i] : c[i - 1];
      const double right = (i + 1 < n_) ? c[i + 1] : c[i];
      return eps * kAxialDispersion * (left - 2.0 * c[i] + right) / (dz_ * dz_);
    };
    double dcco2 = upwind(c_co2_old, inlet.c_co2) + diffuse(c_co2_old) -
                   rho_bulk_ * dqco2;
    double dch2o = upwind(c_h2o_old, inlet.c_h2o) + diffuse(c_h2o_old) -
                   rho_bulk_ * dqh2o;

    // ---- Solid energy balance (explicit; large heat capacity) ----
    const double heat_ads = rho_bulk_ *
        (co2_iso_.heat_of_adsorption() * dqco2 + h2o_iso_.heat_of_adsorption() * dqh2o);
    double q_heater_vol = 0.0;
    if ((mode == BedMode::DESORBING || mode == BedMode::AIR_SAVE) &&
        heater_power_w > 0.0 && ts_old[i] < kHeaterMaxTempK) {
      q_heater_vol = heater_power_w / geom_.volume();
    }
    const double dts = (exch * (tg_old[i] - ts_old[i]) + heat_ads + q_heater_vol) /
                       cap_s;

    // ---- Gas energy balance (semi-implicit in Tg for stiff exchange) ----
    // Explicit advection + axial conduction:
    double adv_t = 0.0;
    if (flowing && v > 0.0) {
      const double tg_up = (i == 0) ? inlet.temperature_k : tg_old[i - 1];
      adv_t = -rho_g * kGasCp * v * (tg_old[i] - tg_up) / dz_;
    }
    const double tg_left = (i == 0) ? tg_old[i] : tg_old[i - 1];
    const double tg_right = (i + 1 < n_) ? tg_old[i + 1] : tg_old[i];
    const double cond_t = thermal_.axial_thermal_cond *
                          (tg_left - 2.0 * tg_old[i] + tg_right) / (dz_ * dz_);
    // Implicit treatment of exch*(Ts-Tg) and wall*(Tg-Twall):
    //   cap_g (Tg_new - Tg)/h = adv + cond + exch (Ts_new - Tg_new)
    //                           - u_wall (Tg_new - Twall)
    const double ts_new = ts_old[i] + h * dts;
    const double rhs = cap_g / h * tg_old[i] + adv_t + cond_t +
                       exch * ts_new + u_wall * thermal_.wall_temp;
    const double denom = cap_g / h + exch + u_wall;
    const double tg_new = rhs / denom;

    // ---- Commit ----
    if (pumping) {
      // The vacuum holds the void gas at the vacuum partial pressure; desorbed
      // gas is swept out by the pump rather than accumulating and re-adsorbing.
      c_co2_[i] = c_vac;
      c_h2o_[i] = c_vac;
    } else {
      c_co2_[i] = std::max(0.0, c_co2_old[i] + h * dcco2 / eps);
      c_h2o_[i] = std::max(0.0, c_h2o_old[i] + h * dch2o / eps);
    }
    q_co2_[i] = std::max(0.0, q_co2_[i] + h * dqco2);
    q_h2o_[i] = std::max(0.0, q_h2o_[i] + h * dqh2o);
    ts_[i] = ts_new;
    tg_[i] = tg_new;
  }
}

BedOutputs BedModel::step(double dt, const BedInlet & inlet, BedMode mode,
                          double heater_power_w)
{
  // Choose a stable substep count from the advection CFL and diffusion limits.
  // The stiff gas-solid energy exchange is handled implicitly, so it does not
  // constrain the substep size.
  const double v_transport = std::abs(inlet.velocity_superficial) / geom_.voidage;
  std::size_t n_sub = numerical::stable_substeps(dt, v_transport, dz_,
                                                 kAxialDispersion, 0.4, 0.4);
  n_sub = std::min(n_sub, kMaxSubsteps);
  const double h = dt / static_cast<double>(n_sub);

  for (std::size_t s = 0; s < n_sub; ++s) {
    integrate_substep(h, inlet, mode, heater_power_w);
  }
  return snapshot(inlet);
}

BedOutputs BedModel::snapshot(const BedInlet & inlet) const
{
  BedOutputs out{};
  out.outlet_c_co2 = c_co2_.back();
  out.outlet_c_h2o = c_h2o_.back();
  out.outlet_temperature = tg_.back();
  out.max_solid_temp = max_solid_temperature();
  out.mean_solid_temp = mean_solid_temperature();
  out.co2_loading_mol = total_co2_loading_mol();
  out.h2o_loading_mol = total_h2o_loading_mol();
  out.pressure_drop_pa = bed_pressure_drop(inlet);
  // Instantaneous CO2 captured = inlet flux - outlet flux over the cross-section.
  const double area = geom_.cross_area();
  const double v = inlet.velocity_superficial;
  out.co2_capture_rate = std::max(0.0, v * area * (inlet.c_co2 - c_co2_.back()));
  return out;
}

}  // namespace ars
}  // namespace ssos_eclss
