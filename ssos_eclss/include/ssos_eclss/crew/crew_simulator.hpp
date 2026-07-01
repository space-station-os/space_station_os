#ifndef SSOS_ECLSS__CREW__CREW_SIMULATOR_HPP_
#define SSOS_ECLSS__CREW__CREW_SIMULATOR_HPP_

#include <string>
#include <vector>

// Astronaut (crew) simulator. Models a crew of N over a 24 h day using a
// diurnal activity schedule (sleep / nominal / exercise / recovery). Metabolic
// GAS rates (CO2 out, O2 consumed, latent water vapour) vary with activity per
// the ICES-2021-365 metabolic tables; the WATER streams (urine, potable draw,
// lost waste water) are steady daily averages per the ISS water balance
// (AIAA water-balance paper, Fig. 1). No ROS, no external deps.

namespace ssos_eclss
{
namespace crew
{

/// One activity phase of the daily schedule (per crew member).
struct CrewActivity
{
  std::string name;      // "sleep", "nominal", "exercise", "recovery"
  double duration_hr;    // hours of the 24 h day in this activity
  double co2_g_min;      // CO2 produced [g/min/person]
  double o2_g_min;       // O2 consumed [g/min/person]
  double latent_g_min;   // respiration + perspiration water vapour [g/min/person]
  double metabolic_w;    // metabolic heat [W/person]
};

/// Crew parameters: size, diurnal schedule, and steady daily water streams.
struct CrewParams
{
  int crew_size;
  std::vector<CrewActivity> schedule;   // durations should sum to 24 h

  // Steady per-crew-member daily water streams [kg/day] (ISS water balance):
  double drink_kg_day;        // drinking + hygiene + food prep (from potable bus)
  double flush_kg_day;        // toilet flush water (from potable bus)
  double urine_kg_day;        // urine to WRS/UPA (recovered)
  double feces_water_kg_day;  // faecal water (lost)
  double trash_water_kg_day;  // wet-trash water (lost)
};

/// Instantaneous crew outputs for the whole crew.
struct CrewOutputs
{
  double co2_kg_s;             // CO2 produced -> cabin (+)
  double o2_consumption_kg_s;  // O2 consumed <- cabin
  double latent_water_kg_s;    // humidity (CHX condensate) -> WRS
  double urine_kg_s;           // urine (+ flush) -> WRS
  double potable_demand_kg_s;  // drink + flush drawn from the potable bus
  double waste_water_kg_s;     // faeces + wet trash (lost)
  double metabolic_heat_w;     // total metabolic heat [W]
  std::string activity;        // current activity name
  double co2_kg_day;           // convenience (whole-crew) [kg/day]
  double o2_kg_day;            // convenience (whole-crew) [kg/day]
};

CrewParams default_crew_params();

/// Crew simulator.
class CrewSimulator
{
public:
  explicit CrewSimulator(const CrewParams & params = default_crew_params());

  /// Outputs at a given point in the diurnal cycle.
  /// @param day_fraction position in the day, [0, 1) (wraps automatically)
  CrewOutputs outputs(double day_fraction) const;

  void set_params(const CrewParams & params) { params_ = params; }
  const CrewParams & params() const { return params_; }

private:
  CrewParams params_;
  const CrewActivity & activity_at(double day_fraction) const;
};

}  // namespace crew
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__CREW__CREW_SIMULATOR_HPP_
