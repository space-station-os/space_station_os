#ifndef SSOS_ECLSS__CABIN__CREW_METABOLIC_MODEL_HPP_
#define SSOS_ECLSS__CABIN__CREW_METABOLIC_MODEL_HPP_

#include "ssos_eclss/cabin/cabin_atmosphere.hpp"

// Crew metabolic loads. Per crew member the model produces CO2 and water vapour
// and metabolic heat while consuming O2, scaled by an activity factor. Nominal
// per-person daily figures follow the NASA BVAD. No ROS, no external deps.

namespace ssos_eclss
{
namespace cabin
{

/// Per-crew-member nominal metabolic rates (daily, at activity factor 1.0).
struct CrewProfile
{
  double co2_kg_day;       // CO2 produced [kg/day]
  double o2_kg_day;        // O2 consumed [kg/day]
  double water_kg_day;     // water vapour produced (resp + persp) [kg/day]
  double heat_w;           // sensible+latent metabolic heat [W]
};

/// Metabolic source/sink rates for the cabin.
struct MetabolicLoads
{
  GasFlows flows;          // O2 (negative), CO2 (positive), H2O (positive) [mol/s]
  double heat_w;           // total metabolic heat [W]
  double water_kg_s;       // total water vapour produced [kg/s]
};

/// Crew metabolic model.
class CrewMetabolicModel
{
public:
  /// @param crew_size number of crew members
  /// @param profile   per-member nominal profile
  CrewMetabolicModel(int crew_size, const CrewProfile & profile);

  /// Compute current metabolic loads.
  /// @param activity_factor multiplier on the nominal rates (1.0 = nominal,
  ///                        higher for exercise)
  MetabolicLoads loads(double activity_factor = 1.0) const;

  void set_crew_size(int n) { crew_size_ = n; }
  int crew_size() const { return crew_size_; }
  void set_profile(const CrewProfile & p) { profile_ = p; }
  const CrewProfile & profile() const { return profile_; }

private:
  int crew_size_;
  CrewProfile profile_;
};

/// Default nominal per-member profile (NASA BVAD averages).
CrewProfile default_crew_profile();

}  // namespace cabin
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__CABIN__CREW_METABOLIC_MODEL_HPP_
