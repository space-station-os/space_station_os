#include "ssos_eclss/crew/crew_simulator.hpp"

#include <algorithm>
#include <cmath>

#include "ssos_eclss/common/units.hpp"

namespace ssos_eclss
{
namespace crew
{

namespace
{
constexpr double kGramPerMinToKgPerS = 1.0e-3 / 60.0;  // g/min -> kg/s
}  // namespace

CrewSimulator::CrewSimulator(const CrewParams & params) : params_(params) {}

const CrewActivity & CrewSimulator::activity_at(double day_fraction) const
{
  // Map the day fraction onto the schedule by duration weight, so the cycle is
  // independent of the (tunable) day length.
  double f = day_fraction - std::floor(day_fraction);  // wrap into [0, 1)
  double total_hr = 0.0;
  for (const auto & a : params_.schedule) {
    total_hr += a.duration_hr;
  }
  if (total_hr <= 0.0) {
    return params_.schedule.back();
  }
  const double t = f * total_hr;
  double edge = 0.0;
  for (const auto & a : params_.schedule) {
    edge += a.duration_hr;
    if (t < edge) {
      return a;
    }
  }
  return params_.schedule.back();
}

CrewOutputs CrewSimulator::outputs(double day_fraction) const
{
  const CrewActivity & a = activity_at(day_fraction);
  const double n = static_cast<double>(params_.crew_size);

  CrewOutputs out{};
  out.activity = a.name;
  // Diurnal gas + latent (g/min/person -> kg/s, whole crew).
  out.co2_kg_s = a.co2_g_min * kGramPerMinToKgPerS * n;
  out.o2_consumption_kg_s = a.o2_g_min * kGramPerMinToKgPerS * n;
  out.latent_water_kg_s = a.latent_g_min * kGramPerMinToKgPerS * n;
  out.metabolic_heat_w = a.metabolic_w * n;

  // Steady daily water streams (kg/day/person -> kg/s, whole crew). Flush water
  // is drawn from the potable bus and returns to the WRS mixed with urine.
  const double per = units::kg_per_day_to_kg_per_s(1.0) * n;
  out.urine_kg_s = (params_.urine_kg_day + params_.flush_kg_day) * per;
  out.potable_demand_kg_s = (params_.drink_kg_day + params_.flush_kg_day) * per;
  out.waste_water_kg_s =
    (params_.feces_water_kg_day + params_.trash_water_kg_day) * per;

  out.co2_kg_day = units::kg_per_s_to_kg_per_day(out.co2_kg_s);
  out.o2_kg_day = units::kg_per_s_to_kg_per_day(out.o2_consumption_kg_s);
  return out;
}

CrewParams default_crew_params()
{
  CrewParams p{};
  p.crew_size = 4;
  // Diurnal schedule (durations sum to 24 h). Rates per crew member from the
  // ICES-2021-365 long-exercise metabolic table (g/min): sleep, nominal,
  // 30 min aerobic exercise, and a recovery tail. Reproduces the daily totals
  // ~1.01 kg CO2, ~0.85 kg O2, ~2.2 kg latent water per person.
  p.schedule = {
    {"sleep",    8.0, 0.44, 0.37, 1.08,  88.0},
    {"nominal",  6.0, 0.69, 0.59, 1.33, 139.0},
    {"exercise", 0.5, 5.22, 3.99, 10.0, 968.0},
    {"recovery", 4.0, 0.69, 0.59, 2.00, 139.0},
    {"nominal",  5.5, 0.69, 0.59, 1.33, 139.0},
  };
  // ISS water balance (AIAA water-balance paper, Fig. 1), per person per day.
  p.drink_kg_day = 2.20;        // drinking + hygiene + food prep
  p.flush_kg_day = 0.30;        // toilet flush
  p.urine_kg_day = 1.20;        // urine
  p.feces_water_kg_day = 0.15;  // faecal water (lost)
  p.trash_water_kg_day = 0.20;  // wet-trash water (lost)
  return p;
}

}  // namespace crew
}  // namespace ssos_eclss
