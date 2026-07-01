#include "ssos_eclss/ars/precooler_model.hpp"

#include <algorithm>

#include "ssos_eclss/common/heat_transfer.hpp"

namespace ssos_eclss
{
namespace ars
{

PrecoolerModel::PrecoolerModel(const PrecoolerParams & params) : params_(params) {}

PrecoolerResult PrecoolerModel::solve(double air_inlet_temp,
                                      double coolant_inlet_temp) const
{
  PrecoolerResult r{};
  const double c_air = params_.air_mass_flow * params_.air_cp;
  const double c_cool = params_.coolant_mass_flow * params_.coolant_cp;
  const double c_min = std::min(c_air, c_cool);
  const double c_max = std::max(c_air, c_cool);

  if (c_min <= 0.0) {
    r.air_outlet_temp = air_inlet_temp;
    r.coolant_outlet_temp = coolant_inlet_temp;
    r.heat_duty = 0.0;
    r.effectiveness = 0.0;
    return r;
  }

  const double cr = c_min / c_max;
  const double ntu = heat::ntu(params_.ua, c_min);
  const double eps = heat::effectiveness_counterflow(ntu, cr);

  // Air is the hot stream (gets cooled).
  const double q = heat::hx_heat_duty(eps, c_min, air_inlet_temp, coolant_inlet_temp);

  r.effectiveness = eps;
  r.heat_duty = q;
  r.air_outlet_temp = air_inlet_temp - q / c_air;
  r.coolant_outlet_temp = coolant_inlet_temp + q / c_cool;
  return r;
}

}  // namespace ars
}  // namespace ssos_eclss
