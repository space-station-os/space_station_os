#include "ssos_eps/battery/battery_oru_model.hpp"

#include <algorithm>
#include <cmath>

#include "ssos_eps/common/interpolation.hpp"
#include "ssos_eps/common/units.hpp"

namespace ssos_eps
{
namespace battery
{

namespace
{
constexpr double kCurrentEpsilonA = 1.0e-6;
}  // namespace

BatteryOruModel::BatteryOruModel(const BatteryOruParams & params)
: params_(params)
{
  reset();
}

void BatteryOruModel::reset()
{
  reset(params_.initial_soc, params_.initial_temp_c);
}

void BatteryOruModel::reset(double soc, double temperature_c)
{
  state_ = BatteryOruState{};
  state_.soc = std::clamp(soc, 0.0, 1.0);
  state_.temperature_c = temperature_c;
  state_.mode = BatteryMode::IDLE;
  failed_ = false;
  update_derived(0.0, 0.0);
}

double BatteryOruModel::usable_capacity_ah() const
{
  return params_.capacity_ah * std::clamp(params_.state_of_health, 0.0, 1.0);
}

double BatteryOruModel::open_circuit_voltage(double soc) const
{
  return static_cast<double>(params_.n_series_cells) *
         common::interpolate(params_.soc_points, params_.cell_ocv_v,
                             std::clamp(soc, 0.0, 1.0));
}

double BatteryOruModel::stored_energy_wh(double soc) const
{
  // The OCV table is piecewise linear, so the trapezoid rule over the
  // breakpoints integrates OCV(SOC) exactly.
  soc = std::clamp(soc, 0.0, 1.0);
  const auto & xs = params_.soc_points;
  double integral = 0.0;
  double prev_x = 0.0;
  double prev_v = open_circuit_voltage(0.0);
  for (double x : xs) {
    if (x <= prev_x) {
      continue;
    }
    const double upper = std::min(x, soc);
    const double v = open_circuit_voltage(upper);
    integral += 0.5 * (prev_v + v) * (upper - prev_x);
    prev_x = upper;
    prev_v = v;
    if (upper >= soc) {
      break;
    }
  }
  if (prev_x < soc) {
    const double v = open_circuit_voltage(soc);
    integral += 0.5 * (prev_v + v) * (soc - prev_x);
  }
  return integral * usable_capacity_ah();
}

double BatteryOruModel::discharge_current_limit_a(double dt) const
{
  if (failed_ || state_.soc <= 0.0) {
    return 0.0;
  }
  const double r = params_.internal_resistance_ohm;
  const double ocv = open_circuit_voltage(state_.soc);
  const double v_min = params_.min_cell_voltage_v * params_.n_series_cells;

  double limit = std::max(params_.max_discharge_current_a, 0.0);
  if (r > 0.0) {
    limit = std::min(limit, std::max(ocv - v_min, 0.0) / r);
    // Beyond the maximum power point more current yields less power.
    limit = std::min(limit, ocv / (2.0 * r));
  }
  if (dt > 0.0) {
    limit = std::min(limit, state_.soc * usable_capacity_ah() *
                     units::SECONDS_PER_HOUR / dt);
  }
  return std::max(limit, 0.0);
}

double BatteryOruModel::charge_current_limit_a(double dt) const
{
  if (failed_ || state_.soc >= 1.0) {
    return 0.0;
  }
  const double r = params_.internal_resistance_ohm;
  const double ocv = open_circuit_voltage(state_.soc);
  const double v_max = params_.max_cell_voltage_v * params_.n_series_cells;

  double limit = std::max(params_.max_charge_current_a, 0.0);
  if (r > 0.0) {
    // Constant-voltage taper: terminal voltage may not exceed v_max.
    limit = std::min(limit, std::max(v_max - ocv, 0.0) / r);
  }
  const double eta = std::max(params_.coulombic_efficiency, 1.0e-6);
  if (dt > 0.0) {
    limit = std::min(limit, (1.0 - state_.soc) * usable_capacity_ah() *
                     units::SECONDS_PER_HOUR / (dt * eta));
  }
  return std::max(limit, 0.0);
}

double BatteryOruModel::max_discharge_power_w(double dt) const
{
  const double i = discharge_current_limit_a(dt);
  const double ocv = open_circuit_voltage(state_.soc);
  return std::max(ocv * i - params_.internal_resistance_ohm * i * i, 0.0);
}

double BatteryOruModel::max_charge_power_w(double dt) const
{
  const double i = charge_current_limit_a(dt);
  const double ocv = open_circuit_voltage(state_.soc);
  return ocv * i + params_.internal_resistance_ohm * i * i;
}

double BatteryOruModel::step(double dt, double power_request_w)
{
  dt = std::max(dt, 0.0);
  const double r = params_.internal_resistance_ohm;
  const double ocv = open_circuit_voltage(state_.soc);

  // Solve P = OCV*I - R*I^2 for the current, signed by direction.
  double current = 0.0;
  if (power_request_w > 0.0) {
    const double limit = discharge_current_limit_a(dt);
    double i = power_request_w / std::max(ocv, 1.0e-6);
    if (r > 0.0) {
      const double disc = ocv * ocv - 4.0 * r * power_request_w;
      i = disc > 0.0 ? (ocv - std::sqrt(disc)) / (2.0 * r) : ocv / (2.0 * r);
    }
    current = std::min(i, limit);
  } else if (power_request_w < 0.0) {
    const double p = -power_request_w;
    const double limit = charge_current_limit_a(dt);
    double i = p / std::max(ocv, 1.0e-6);
    if (r > 0.0) {
      i = (-ocv + std::sqrt(ocv * ocv + 4.0 * r * p)) / (2.0 * r);
    }
    current = -std::min(i, limit);
  }

  if (dt > 0.0) {
    const double capacity_as = usable_capacity_ah() * units::SECONDS_PER_HOUR;
    if (capacity_as > 0.0) {
      const double charge_in = current < 0.0 ? params_.coulombic_efficiency : 1.0;
      state_.soc = std::clamp(state_.soc - current * charge_in * dt / capacity_as, 0.0, 1.0);
    }
  }

  state_.power_w = ocv * current - r * current * current;
  state_.terminal_voltage_v = ocv - r * current;
  update_derived(current, dt);
  return state_.power_w;
}

void BatteryOruModel::update_derived(double current_a, double dt)
{
  state_.current_a = current_a;
  state_.heat_w = current_a * current_a * params_.internal_resistance_ohm;
  state_.ocv_v = open_circuit_voltage(state_.soc);
  if (std::abs(current_a) < kCurrentEpsilonA) {
    state_.current_a = 0.0;
    state_.power_w = 0.0;
    state_.terminal_voltage_v = state_.ocv_v;
    state_.mode = BatteryMode::IDLE;
  } else {
    state_.mode = current_a > 0.0 ? BatteryMode::DISCHARGING : BatteryMode::CHARGING;
  }

  if (dt > 0.0 && params_.thermal_mass_j_k > 0.0) {
    // Exact solution of C dT/dt = Q - G (T - T_cp) over dt.
    const double g = std::max(params_.heat_transfer_w_k, 0.0);
    if (g > 0.0) {
      const double t_eq = params_.coldplate_temp_c + state_.heat_w / g;
      const double decay = std::exp(-g * dt / params_.thermal_mass_j_k);
      state_.temperature_c = t_eq + (state_.temperature_c - t_eq) * decay;
    } else {
      state_.temperature_c += state_.heat_w * dt / params_.thermal_mass_j_k;
    }
  }

  state_.stored_energy_wh = stored_energy_wh(state_.soc);
  if (failed_) {
    state_.health = BatteryHealth::FAILED;
  } else if (state_.temperature_c > params_.max_temp_c) {
    state_.health = BatteryHealth::OVERHEAT;
  } else {
    state_.health = BatteryHealth::GOOD;
  }
}

void BatteryOruModel::set_failed(bool failed)
{
  failed_ = failed;
  update_derived(0.0, 0.0);
}

}  // namespace battery
}  // namespace ssos_eps
