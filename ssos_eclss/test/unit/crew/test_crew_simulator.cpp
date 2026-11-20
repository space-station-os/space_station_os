#include <gtest/gtest.h>

#include "ssos_eclss/crew/crew_simulator.hpp"

using ssos_eclss::crew::Activity;
using ssos_eclss::crew::CrewSimulator;
using ssos_eclss::crew::default_crew_params;

namespace
{
// Integrate whole-crew outputs over one day in one-minute steps.
struct DayTotals
{
  double co2_kg = 0.0, o2_kg = 0.0, latent_kg = 0.0;
  int max_exercising = 0;
};

DayTotals integrate_day(const CrewSimulator & crew)
{
  DayTotals t;
  for (int m = 0; m < 1440; ++m) {
    const auto out = crew.outputs(m / 60.0);
    t.co2_kg += out.co2_kg_s * 60.0;
    t.o2_kg += out.o2_consumption_kg_s * 60.0;
    t.latent_kg += out.latent_water_kg_s * 60.0;
    t.max_exercising = std::max(t.max_exercising, out.exercising);
  }
  return t;
}
}  // namespace

TEST(CrewSimulator, DailyTotalsMatchTheMetabolicTables)
{
  CrewSimulator crew;
  const DayTotals t = integrate_day(crew);
  EXPECT_NEAR(t.co2_kg / 4.0, 1.01, 0.02);
  EXPECT_NEAR(t.o2_kg / 4.0, 0.85, 0.02);
  EXPECT_NEAR(t.latent_kg / 4.0, 2.2, 0.05);
}

TEST(CrewSimulator, AtMostOneMemberExercisesAtATime)
{
  CrewSimulator crew;
  EXPECT_EQ(integrate_day(crew).max_exercising, 1);
}

TEST(CrewSimulator, CrewSleepsTogetherAcrossMidnight)
{
  CrewSimulator crew;
  for (double h : {22.0, 23.9, 0.5, 5.0}) {
    const auto out = crew.outputs(h);
    EXPECT_EQ(out.activity, Activity::Sleep) << h;
  }
  EXPECT_NE(crew.outputs(12.0).activity, Activity::Sleep);
}

TEST(CrewSimulator, WaterStreamsFollowTheIssBalance)
{
  CrewSimulator crew;
  const auto out = crew.outputs(12.0);
  EXPECT_NEAR(out.urine_kg_s * 86400.0, 4 * 1.5, 1.0e-9);
  EXPECT_NEAR(out.potable_demand_kg_s * 86400.0, 4 * 2.5, 1.0e-9);
}

TEST(CrewSimulator, MetabolicScaleMultipliesGasAndHeat)
{
  CrewSimulator crew;
  const auto base = crew.outputs(12.0);
  const auto surge = crew.outputs(12.0, 2.0);
  EXPECT_NEAR(surge.co2_kg_s, 2.0 * base.co2_kg_s, 1.0e-12);
  EXPECT_NEAR(surge.metabolic_heat_w, 2.0 * base.metabolic_heat_w, 1.0e-9);
  EXPECT_DOUBLE_EQ(surge.urine_kg_s, base.urine_kg_s);
}
