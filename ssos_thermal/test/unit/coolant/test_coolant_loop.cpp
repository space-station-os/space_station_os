#include <gtest/gtest.h>

#include "ssos_thermal/coolant/coolant_loop.hpp"

using ssos_thermal::coolant::CoolantLoop;
using ssos_thermal::coolant::CoolantParams;

TEST(CoolantLoopTest, StepMovesTemperatureTowardTargetByAtMost2Point5Deg)
{
  CoolantLoop loop{CoolantParams{}};
  const auto result = loop.step(40.0, 25.0);
  EXPECT_DOUBLE_EQ(result.node_temp_c, 37.5);
  EXPECT_FALSE(result.done);
}

TEST(CoolantLoopTest, StepClampsToRemainingDeltaNearTarget)
{
  CoolantLoop loop{CoolantParams{}};
  // Only 0.3 degC above target -- delta_T should be capped at 0.3, not 2.5.
  const auto result = loop.step(25.3, 25.0);
  EXPECT_NEAR(result.node_temp_c, 25.0, 1e-9);
  EXPECT_TRUE(result.done);
}

TEST(CoolantLoopTest, VentTriggersOnceAmmoniaHeatCrossesThreshold)
{
  CoolantParams params;
  params.vent_threshold_kj = 1.0;  // low threshold so a normal step trips it
  CoolantLoop loop{params};
  const auto result = loop.step(40.0, 25.0);
  EXPECT_TRUE(result.vent_triggered);
  EXPECT_GE(result.ammonia_heat_kj, params.vent_threshold_kj);
}

TEST(CoolantLoopTest, NoVentBelowThreshold)
{
  CoolantParams params;
  params.vent_threshold_kj = 1.0e9;  // effectively unreachable
  CoolantLoop loop{params};
  const auto result = loop.step(40.0, 25.0);
  EXPECT_FALSE(result.vent_triggered);
}

TEST(CoolantLoopTest, RepeatedStepsConvergeToTarget)
{
  CoolantLoop loop{CoolantParams{}};
  double temp = 60.0;
  int iterations = 0;
  while (temp > 25.0 + 0.5 && iterations < 1000) {
    const auto result = loop.step(temp, 25.0);
    temp = result.node_temp_c;
    ++iterations;
  }
  EXPECT_NEAR(temp, 25.0, 0.5);
  EXPECT_LT(iterations, 1000);
}
