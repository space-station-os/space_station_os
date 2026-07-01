#include <gtest/gtest.h>

#include <string>

#include "ssos_eclss/crew/crew_simulator.hpp"

using namespace ssos_eclss::crew;

namespace
{
constexpr double kSecondsPerDay = 86400.0;

// Day fractions landing inside specific schedule phases (default schedule:
// sleep 0-8h, nominal 8-14h, exercise 14-14.5h, recovery 14.5-18.5h, nominal ..).
constexpr double kSleepFrac = 0.5 / 24.0;      // 00:30
constexpr double kExerciseFrac = 14.25 / 24.0;  // 14:15

// Numerically integrate a whole-crew rate over one day -> kg/day.
double daily_total(const CrewSimulator & sim,
                   double (*pick)(const CrewOutputs &))
{
  const int n = 20000;
  double acc = 0.0;
  for (int i = 0; i < n; ++i) {
    acc += pick(sim.outputs((i + 0.5) / n));
  }
  return acc / n * kSecondsPerDay;
}
}  // namespace

TEST(CrewSimulator, ActivityPhasesByTimeOfDay)
{
  CrewSimulator sim;
  EXPECT_EQ(sim.outputs(kSleepFrac).activity, "sleep");
  EXPECT_EQ(sim.outputs(kExerciseFrac).activity, "exercise");
}

TEST(CrewSimulator, ExerciseProducesMoreThanSleep)
{
  CrewSimulator sim;
  EXPECT_GT(sim.outputs(kExerciseFrac).co2_kg_s, sim.outputs(kSleepFrac).co2_kg_s);
  EXPECT_GT(sim.outputs(kExerciseFrac).o2_consumption_kg_s,
            sim.outputs(kSleepFrac).o2_consumption_kg_s);
  EXPECT_GT(sim.outputs(kExerciseFrac).latent_water_kg_s,
            sim.outputs(kSleepFrac).latent_water_kg_s);
}

TEST(CrewSimulator, DailyTotalsMatchICES2021)
{
  CrewSimulator sim;  // default 4 crew
  const double co2 = daily_total(sim, [](const CrewOutputs & o) { return o.co2_kg_s; });
  const double o2 =
    daily_total(sim, [](const CrewOutputs & o) { return o.o2_consumption_kg_s; });
  const double latent =
    daily_total(sim, [](const CrewOutputs & o) { return o.latent_water_kg_s; });
  // Paper per-person daily totals ~1.01 CO2, ~0.85 O2, ~2.2 latent; x4 crew.
  EXPECT_NEAR(co2 / 4.0, 1.01, 0.10);
  EXPECT_NEAR(o2 / 4.0, 0.85, 0.10);
  EXPECT_NEAR(latent / 4.0, 2.2, 0.30);
}

TEST(CrewSimulator, ScalesWithCrewSize)
{
  CrewParams p1 = default_crew_params();
  p1.crew_size = 1;
  CrewParams p4 = default_crew_params();
  p4.crew_size = 4;
  const double c1 = CrewSimulator(p1).outputs(kSleepFrac).co2_kg_s;
  const double c4 = CrewSimulator(p4).outputs(kSleepFrac).co2_kg_s;
  EXPECT_NEAR(c4, 4.0 * c1, 1e-12);
}

TEST(CrewSimulator, SteadyWaterStreams)
{
  CrewSimulator sim;  // 4 crew
  const CrewOutputs o = sim.outputs(kSleepFrac);
  const double per_day = 86400.0;
  // urine = (1.20 + 0.30 flush) x4; potable = (2.20 + 0.30) x4; waste = (0.15+0.20) x4.
  EXPECT_NEAR(o.urine_kg_s * per_day, (1.20 + 0.30) * 4.0, 1e-6);
  EXPECT_NEAR(o.potable_demand_kg_s * per_day, (2.20 + 0.30) * 4.0, 1e-6);
  EXPECT_NEAR(o.waste_water_kg_s * per_day, (0.15 + 0.20) * 4.0, 1e-6);
}

TEST(CrewSimulator, DayFractionWraps)
{
  CrewSimulator sim;
  EXPECT_EQ(sim.outputs(0.25).activity, sim.outputs(1.25).activity);
  EXPECT_NEAR(sim.outputs(0.25).co2_kg_s, sim.outputs(2.25).co2_kg_s, 1e-12);
}

TEST(CrewSimulator, MetabolicScaleAffectsGasNotWater)
{
  CrewParams p = default_crew_params();
  for (auto & a : p.schedule) {
    a.co2_g_min *= 1.5;
    a.o2_g_min *= 1.5;
    a.latent_g_min *= 1.5;
  }
  CrewSimulator scaled(p);
  CrewSimulator nominal;
  EXPECT_NEAR(scaled.outputs(kSleepFrac).co2_kg_s,
              1.5 * nominal.outputs(kSleepFrac).co2_kg_s, 1e-12);
  // Water streams are independent of the metabolic (gas) schedule.
  EXPECT_NEAR(scaled.outputs(kSleepFrac).urine_kg_s,
              nominal.outputs(kSleepFrac).urine_kg_s, 1e-12);
}
