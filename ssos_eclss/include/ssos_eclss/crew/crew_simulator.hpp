#ifndef SSOS_ECLSS__CREW__CREW_SIMULATOR_HPP_
#define SSOS_ECLSS__CREW__CREW_SIMULATOR_HPP_

// Crew metabolic and water loads over a 24 h day. The crew sleeps together;
// each member exercises once a day in a staggered slot and recovers
// afterwards, so at most one member is exercising at a time. Gas and latent
// water rates per activity follow ICES-2021-365; the water streams are daily
// averages from the ISS water balance. No ROS.

namespace ssos_eclss
{
namespace crew
{

enum class Activity
{
  Sleep,
  Nominal,
  Recovery,
  Exercise,   // ordered by metabolic intensity
};

const char * to_string(Activity activity);

/// Per-person rates during one activity.
struct ActivityRates
{
  double co2_g_min;      // CO2 produced [g/min]
  double o2_g_min;       // O2 consumed [g/min]
  double latent_g_min;   // respiration + perspiration water vapour [g/min]
  double metabolic_w;    // metabolic heat [W]
};

struct CrewParams
{
  int crew_size;

  double sleep_start_hr;        // crew day: sleep window start (may wrap midnight)
  double sleep_hours;
  double first_exercise_hr;     // member 0 starts exercising here
  double exercise_spacing_hr;   // next member starts this much later
  double exercise_hours;
  double recovery_hours;        // elevated latent load after exercise

  ActivityRates sleep;
  ActivityRates nominal;
  ActivityRates exercise;
  ActivityRates recovery;

  // Per person per day [kg/day].
  double drink_kg_day;          // drinking, hygiene, food preparation (potable bus)
  double flush_kg_day;          // toilet flush (potable bus, returns with urine)
  double urine_kg_day;
  double feces_water_kg_day;    // lost
  double trash_water_kg_day;    // lost
};

/// Whole-crew loads at one time of day.
struct CrewOutputs
{
  double co2_kg_s;
  double o2_consumption_kg_s;
  double latent_water_kg_s;     // water vapour into the cabin air
  double urine_kg_s;            // urine + flush water to the WRS
  double potable_demand_kg_s;   // drink + flush from the potable bus
  double waste_water_kg_s;      // faeces + wet trash water (lost)
  double metabolic_heat_w;
  Activity activity;            // most intense activity aboard
  int exercising;               // members exercising now
};

CrewParams default_crew_params();

class CrewSimulator
{
public:
  explicit CrewSimulator(const CrewParams & params = default_crew_params());

  void set_params(const CrewParams & params) {params_ = params;}
  const CrewParams & params() const {return params_;}

  Activity member_activity(int member, double hour_of_day) const;

  /// @param hour_of_day crew time [h], wrapped into [0, 24)
  /// @param metabolic_scale multiplier on the gas, latent and heat rates
  CrewOutputs outputs(double hour_of_day, double metabolic_scale = 1.0) const;

private:
  const ActivityRates & rates(Activity activity) const;

  CrewParams params_;
};

}  // namespace crew
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__CREW__CREW_SIMULATOR_HPP_
