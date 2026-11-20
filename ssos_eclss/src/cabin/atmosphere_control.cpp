#include "ssos_eclss/cabin/atmosphere_control.hpp"

#include <algorithm>

#include "ssos_eclss/common/gas_properties.hpp"
#include "ssos_eclss/common/units.hpp"

namespace ssos_eclss
{
namespace cabin
{

AtmosphereControl::AtmosphereControl(const AtmosphereControlParams & params)
: params_(params)
{
  reset();
}

void AtmosphereControl::reset()
{
  n2_tank_kg_ = params_.n2_tank_kg;
  o2_tank_kg_ = params_.o2_tank_kg;
  n2_makeup_on_ = false;
  chx_failed_ = false;
}

ControlResult AtmosphereControl::step(double dt, const CabinAtmosphere & atm)
{
  ControlResult r{};
  if (dt <= 0.0) {
    return r;
  }
  const double moles_per_pa = atm.volume_m3() / (units::R_GAS * atm.temperature_k());

  // CHX: remove the vapour above the setpoint over its time constant.
  if (!chx_failed_) {
    const double pp_set = gas::water_pp_from_rh(params_.rh_setpoint, atm.temperature_k());
    const double excess_mol =
      (atm.partial_pressure_pa(Gas::H2O) - pp_set) * moles_per_pa;
    const double max_mol_s =
      units::kg_per_day_to_kg_per_s(params_.chx_max_condensate_kg_day) / units::M_H2O;
    const double removal = std::clamp(
      excess_mol / std::max(params_.chx_time_constant_s, dt), 0.0, max_mol_s);
    r.flows.h2o = -removal;
    r.condensate_kg_s = removal * units::M_H2O;
  }

  const double max_makeup_kg_s = units::kg_per_day_to_kg_per_s(params_.makeup_max_kg_day);

  // N2 makeup with hysteresis: on below the low limit, off at the setpoint.
  const double p_kpa = units::pa_to_kpa(atm.total_pressure_pa());
  if (p_kpa < params_.pressure_low_kpa) {
    n2_makeup_on_ = true;
  } else if (p_kpa >= params_.pressure_setpoint_kpa) {
    n2_makeup_on_ = false;
  }
  if (n2_makeup_on_ && n2_tank_kg_ > 0.0) {
    const double deficit_kg = units::kpa_to_pa(params_.pressure_setpoint_kpa - p_kpa) *
      moles_per_pa * units::M_N2;
    r.n2_makeup_kg_s = std::min({max_makeup_kg_s, std::max(deficit_kg, 0.0) / dt,
        n2_tank_kg_ / dt});
    n2_tank_kg_ -= r.n2_makeup_kg_s * dt;
    r.flows.n2 = r.n2_makeup_kg_s / units::M_N2;
  }

  // O2 makeup only below the floor.
  const double o2_kpa = units::pa_to_kpa(atm.partial_pressure_pa(Gas::O2));
  if (o2_kpa < params_.o2_floor_kpa && o2_tank_kg_ > 0.0) {
    const double deficit_kg = units::kpa_to_pa(params_.o2_floor_kpa - o2_kpa) *
      moles_per_pa * units::M_O2;
    r.o2_makeup_kg_s = std::min({max_makeup_kg_s, deficit_kg / dt, o2_tank_kg_ / dt});
    o2_tank_kg_ -= r.o2_makeup_kg_s * dt;
    r.flows.o2 = r.o2_makeup_kg_s / units::M_O2;
  }
  return r;
}

AtmosphereControlParams default_atmosphere_control_params()
{
  AtmosphereControlParams p{};
  p.rh_setpoint = 0.45;
  p.chx_time_constant_s = 900.0;
  p.chx_max_condensate_kg_day = 30.0;
  p.pressure_setpoint_kpa = 101.3;
  p.pressure_low_kpa = 100.6;
  p.o2_floor_kpa = 19.5;
  p.makeup_max_kg_day = 50.0;
  p.n2_tank_kg = 200.0;
  p.o2_tank_kg = 200.0;
  return p;
}

}  // namespace cabin
}  // namespace ssos_eclss
