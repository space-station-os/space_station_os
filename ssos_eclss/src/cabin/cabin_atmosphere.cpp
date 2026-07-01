#include "ssos_eclss/cabin/cabin_atmosphere.hpp"

#include <algorithm>

#include "ssos_eclss/common/gas_properties.hpp"
#include "ssos_eclss/common/units.hpp"

namespace ssos_eclss
{
namespace cabin
{

CabinAtmosphere::CabinAtmosphere(const CabinParams & params) : params_(params)
{
  initialize_nominal();
}

void CabinAtmosphere::initialize_nominal(double co2_ppm, double relative_humidity)
{
  const double t = params_.temperature_k;
  const double v = params_.volume_m3;
  const double total_p = units::STD_PRESSURE_PA;

  // Water vapour from RH.
  const double pp_h2o = gas::water_pp_from_rh(relative_humidity, t);
  // CO2 from ppm of total pressure.
  const double pp_co2 = units::ppm_to_fraction(co2_ppm) * total_p;
  // O2 at ~21% of the dry-gas pressure.
  const double dry_p = total_p - pp_h2o - pp_co2;
  const double pp_o2 = 0.21 * dry_p;
  const double pp_n2 = dry_p - pp_o2;

  auto moles_for = [&](double pp) { return pp * v / (units::R_GAS * t); };
  n_o2_ = moles_for(pp_o2);
  n_co2_ = moles_for(pp_co2);
  n_n2_ = moles_for(pp_n2);
  n_h2o_ = moles_for(pp_h2o);
}

double & CabinAtmosphere::ref(Gas species)
{
  switch (species) {
    case Gas::O2: return n_o2_;
    case Gas::CO2: return n_co2_;
    case Gas::N2: return n_n2_;
    case Gas::H2O: return n_h2o_;
  }
  return n_n2_;
}

double CabinAtmosphere::get(Gas species) const
{
  switch (species) {
    case Gas::O2: return n_o2_;
    case Gas::CO2: return n_co2_;
    case Gas::N2: return n_n2_;
    case Gas::H2O: return n_h2o_;
  }
  return 0.0;
}

void CabinAtmosphere::apply_flows(double dt, const GasFlows & flows)
{
  n_o2_ = std::max(0.0, n_o2_ + flows.o2 * dt);
  n_co2_ = std::max(0.0, n_co2_ + flows.co2 * dt);
  n_n2_ = std::max(0.0, n_n2_ + flows.n2 * dt);
  n_h2o_ = std::max(0.0, n_h2o_ + flows.h2o * dt);
}

void CabinAtmosphere::add_moles(Gas species, double moles)
{
  double & r = ref(species);
  r = std::max(0.0, r + moles);
}

double CabinAtmosphere::total_moles() const
{
  return n_o2_ + n_co2_ + n_n2_ + n_h2o_;
}

double CabinAtmosphere::moles(Gas species) const { return get(species); }

double CabinAtmosphere::total_pressure_pa() const
{
  return total_moles() * units::R_GAS * params_.temperature_k / params_.volume_m3;
}

double CabinAtmosphere::partial_pressure_pa(Gas species) const
{
  return get(species) * units::R_GAS * params_.temperature_k / params_.volume_m3;
}

double CabinAtmosphere::mole_fraction(Gas species) const
{
  const double tot = total_moles();
  return (tot > 0.0) ? get(species) / tot : 0.0;
}

double CabinAtmosphere::co2_ppm() const
{
  return units::fraction_to_ppm(mole_fraction(Gas::CO2));
}

double CabinAtmosphere::o2_fraction() const { return mole_fraction(Gas::O2); }

double CabinAtmosphere::relative_humidity() const
{
  return gas::relative_humidity(partial_pressure_pa(Gas::H2O), params_.temperature_k);
}

double CabinAtmosphere::dew_point_k() const
{
  return gas::dew_point_from_pp(partial_pressure_pa(Gas::H2O));
}

double CabinAtmosphere::total_gas_mass_kg() const
{
  return n_o2_ * units::M_O2 + n_co2_ * units::M_CO2 + n_n2_ * units::M_N2 +
         n_h2o_ * units::M_H2O;
}

}  // namespace cabin
}  // namespace ssos_eclss
