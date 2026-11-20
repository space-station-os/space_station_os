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
constexpr double kGramPerMinToKgPerS = 1.0e-3 / 60.0;
constexpr double kHoursPerDay = 24.0;

double wrap_hours(double h)
{
  h = std::fmod(h, kHoursPerDay);
  return h < 0.0 ? h + kHoursPerDay : h;
}

// True if hour lies in [start, start + length), with wrap-around at midnight.
bool in_window(double hour, double start, double length)
{
  return wrap_hours(hour - start) < length;
}
}  // namespace

const char * to_string(Activity activity)
{
  switch (activity) {
    case Activity::Sleep: return "sleep";
    case Activity::Nominal: return "nominal";
    case Activity::Recovery: return "recovery";
    case Activity::Exercise: return "exercise";
  }
  return "nominal";
}

CrewSimulator::CrewSimulator(const CrewParams & params) : params_(params) {}

const ActivityRates & CrewSimulator::rates(Activity activity) const
{
  switch (activity) {
    case Activity::Sleep: return params_.sleep;
    case Activity::Recovery: return params_.recovery;
    case Activity::Exercise: return params_.exercise;
    case Activity::Nominal: break;
  }
  return params_.nominal;
}

Activity CrewSimulator::member_activity(int member, double hour_of_day) const
{
  const double h = wrap_hours(hour_of_day);
  if (in_window(h, params_.sleep_start_hr, params_.sleep_hours)) {
    return Activity::Sleep;
  }
  const double start = params_.first_exercise_hr + member * params_.exercise_spacing_hr;
  if (in_window(h, start, params_.exercise_hours)) {
    return Activity::Exercise;
  }
  if (in_window(h, start + params_.exercise_hours, params_.recovery_hours)) {
    return Activity::Recovery;
  }
  return Activity::Nominal;
}

CrewOutputs CrewSimulator::outputs(double hour_of_day, double metabolic_scale) const
{
  CrewOutputs out{};
  out.activity = Activity::Sleep;
  double co2_g_min = 0.0;
  double o2_g_min = 0.0;
  double latent_g_min = 0.0;
  for (int m = 0; m < params_.crew_size; ++m) {
    const Activity a = member_activity(m, hour_of_day);
    const ActivityRates & r = rates(a);
    co2_g_min += r.co2_g_min;
    o2_g_min += r.o2_g_min;
    latent_g_min += r.latent_g_min;
    out.metabolic_heat_w += r.metabolic_w;
    out.activity = std::max(out.activity, a);
    out.exercising += a == Activity::Exercise ? 1 : 0;
  }
  const double scale = std::max(metabolic_scale, 0.0) * kGramPerMinToKgPerS;
  out.co2_kg_s = co2_g_min * scale;
  out.o2_consumption_kg_s = o2_g_min * scale;
  out.latent_water_kg_s = latent_g_min * scale;
  out.metabolic_heat_w *= std::max(metabolic_scale, 0.0);

  const double per_day = units::kg_per_day_to_kg_per_s(1.0) * params_.crew_size;
  out.urine_kg_s = (params_.urine_kg_day + params_.flush_kg_day) * per_day;
  out.potable_demand_kg_s = (params_.drink_kg_day + params_.flush_kg_day) * per_day;
  out.waste_water_kg_s = (params_.feces_water_kg_day + params_.trash_water_kg_day) * per_day;
  return out;
}

CrewParams default_crew_params()
{
  CrewParams p{};
  p.crew_size = 4;
  p.sleep_start_hr = 21.5;
  p.sleep_hours = 8.0;
  p.first_exercise_hr = 8.0;
  p.exercise_spacing_hr = 2.0;
  p.exercise_hours = 0.5;
  p.recovery_hours = 4.0;
  // ICES-2021-365 long-exercise table, per person. Over the day these give
  // about 1.01 kg CO2, 0.85 kg O2 and 2.2 kg latent water per person.
  p.sleep = {0.44, 0.37, 1.08, 88.0};
  p.nominal = {0.69, 0.59, 1.33, 139.0};
  p.exercise = {5.22, 3.99, 10.0, 968.0};
  p.recovery = {0.69, 0.59, 2.00, 139.0};
  // ISS water balance, per person per day.
  p.drink_kg_day = 2.20;
  p.flush_kg_day = 0.30;
  p.urine_kg_day = 1.20;
  p.feces_water_kg_day = 0.15;
  p.trash_water_kg_day = 0.20;
  return p;
}

}  // namespace crew
}  // namespace ssos_eclss
