#include "ssos_eclss/ars/air_save_pump_model.hpp"

#include <algorithm>
#include <cmath>

#include "ssos_eclss/common/units.hpp"

namespace ssos_eclss
{
namespace ars
{

AirSavePumpModel::AirSavePumpModel(const AirSavePumpParams & params) : params_(params)
{}

AirSaveResult AirSavePumpModel::pump(double dt, double bed_pressure_pa,
                                     double temperature_k) const
{
  AirSaveResult r{};

  // Volumetric pump on a fixed volume: P decays exponentially toward the
  // ultimate pressure with time constant tau = V / S.
  const double tau = params_.void_volume_m3 / std::max(params_.displacement_m3s, 1.0e-9);
  const double p0 = bed_pressure_pa;
  const double p_ult = params_.ultimate_pressure_pa;
  const double p_new = p_ult + (p0 - p_ult) * std::exp(-dt / tau);

  // Moles removed from the void = V/(RT) * (P0 - P_new). These are recovered to
  // the cabin (air-save), only the residual below ultimate pressure is lost.
  const double dn = params_.void_volume_m3 / (units::R_GAS * temperature_k) *
                    std::max(p0 - p_new, 0.0);
  const double n_initial = params_.void_volume_m3 / (units::R_GAS * temperature_k) * p0;

  r.bed_pressure_pa = p_new;
  r.air_recovered_mol = dn;
  r.recovery_fraction = (n_initial > 0.0) ? dn / n_initial : 0.0;
  return r;
}

}  // namespace ars
}  // namespace ssos_eclss
