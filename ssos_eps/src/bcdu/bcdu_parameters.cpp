#include "ssos_eps/bcdu/bcdu_parameters.hpp"

namespace ssos_eps
{
namespace bcdu
{

BcduParams default_bcdu_params()
{
  BcduParams p{};
  // The legacy controller switched at 160 V, which sits on the SSU setpoint
  // and chatters with its ripple. 158 V keeps the same 152 V lower bound while
  // staying clear of the regulated band.
  p.charge_enter_v = 158.0;
  p.discharge_enter_v = 152.0;
  p.regulation_voltage_v = 150.0;
  p.regulation_min_v = 130.0;
  p.regulation_max_v = 180.0;
  p.max_charge_current_a = 65.0;
  p.max_discharge_current_a = 127.0;
  p.charge_efficiency = 0.95;
  p.discharge_efficiency = 0.95;
  p.deadband_w = 10.0;
  p.unserved_tolerance_w = 1.0;
  return p;
}

}  // namespace bcdu
}  // namespace ssos_eps
