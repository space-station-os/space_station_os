#include "ssos_eclss/cabin/crew_metabolic_model.hpp"

#include "ssos_eclss/common/units.hpp"

namespace ssos_eclss
{
namespace cabin
{

CrewMetabolicModel::CrewMetabolicModel(int crew_size, const CrewProfile & profile)
: crew_size_(crew_size), profile_(profile)
{}

MetabolicLoads CrewMetabolicModel::loads(double activity_factor) const
{
  MetabolicLoads out{};
  const double n = static_cast<double>(crew_size_) * activity_factor;

  // Convert per-day mass rates to per-second molar rates.
  const double co2_kg_s = units::kg_per_day_to_kg_per_s(profile_.co2_kg_day) * n;
  const double o2_kg_s = units::kg_per_day_to_kg_per_s(profile_.o2_kg_day) * n;
  const double h2o_kg_s = units::kg_per_day_to_kg_per_s(profile_.water_kg_day) * n;

  out.flows.co2 = co2_kg_s / units::M_CO2;     // produced (+)
  out.flows.o2 = -o2_kg_s / units::M_O2;       // consumed (-)
  out.flows.h2o = h2o_kg_s / units::M_H2O;     // produced (+)
  out.flows.n2 = 0.0;

  out.heat_w = profile_.heat_w * n;
  out.water_kg_s = h2o_kg_s;
  return out;
}

CrewProfile default_crew_profile()
{
  CrewProfile p;
  p.co2_kg_day = 1.04;    // kg/day CO2
  p.o2_kg_day = 0.84;     // kg/day O2
  p.water_kg_day = 2.28;  // kg/day respiration + perspiration vapour
  p.heat_w = 117.0;       // W average metabolic heat
  return p;
}

}  // namespace cabin
}  // namespace ssos_eclss
