#ifndef SSOS_ECLSS__CABIN__ATMOSPHERE_CONTROL_HPP_
#define SSOS_ECLSS__CABIN__ATMOSPHERE_CONTROL_HPP_

#include "ssos_eclss/cabin/cabin_atmosphere.hpp"

// Cabin humidity and pressure control. The condensing heat exchanger (CHX)
// removes water vapour toward a relative-humidity setpoint, limited by its
// capacity; the condensate goes to the water processor. Pressure control
// (PCA) adds N2 from stored tanks when total pressure falls below its band,
// and O2 only if ppO2 falls below its floor (normally the OGS holds ppO2).
// No ROS.

namespace ssos_eclss
{
namespace cabin
{

struct AtmosphereControlParams
{
  double rh_setpoint;             // CHX target relative humidity [0, 1]
  double chx_time_constant_s;     // time to remove an excess of vapour [s]
  double chx_max_condensate_kg_day;
  double pressure_setpoint_kpa;   // N2 makeup restores this total pressure
  double pressure_low_kpa;        // N2 makeup starts below this
  double o2_floor_kpa;            // O2 makeup below this ppO2
  double makeup_max_kg_day;       // per gas
  double n2_tank_kg;              // initial stored N2
  double o2_tank_kg;              // initial stored O2
};

AtmosphereControlParams default_atmosphere_control_params();

struct ControlResult
{
  GasFlows flows;                 // [mol/s] to add to the cabin (condensate negative)
  double condensate_kg_s;
  double n2_makeup_kg_s;
  double o2_makeup_kg_s;
};

class AtmosphereControl
{
public:
  explicit AtmosphereControl(
    const AtmosphereControlParams & params = default_atmosphere_control_params());

  void reset();
  void set_chx_failed(bool failed) {chx_failed_ = failed;}

  /// Control flows for the next dt seconds, drawing makeup gas from the tanks.
  ControlResult step(double dt, const CabinAtmosphere & atm);

  double n2_tank_kg() const {return n2_tank_kg_;}
  double o2_tank_kg() const {return o2_tank_kg_;}

private:
  AtmosphereControlParams params_;
  double n2_tank_kg_{0.0};
  double o2_tank_kg_{0.0};
  bool n2_makeup_on_{false};
  bool chx_failed_{false};
};

}  // namespace cabin
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__CABIN__ATMOSPHERE_CONTROL_HPP_
