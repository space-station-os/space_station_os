#include <gtest/gtest.h>

#include "ssos_eclss/ogs/o2_controller.hpp"

using ssos_eclss::ogs::O2PressureController;
using ssos_eclss::ogs::default_o2_controller_params;

namespace
{
// Cabin ppO2 response: 1 A of stack current makes about 0.197 kg O2/day; in a
// 388 m^3 cabin 1 kg/day of imbalance moves ppO2 by about 0.197 kPa/day.
constexpr double kKpaPerSecPerAmp = 0.197 * 0.197 / 86400.0;
constexpr double kKpaPerSecPerKgDay = 0.197 / 86400.0;
}

TEST(O2Controller, BumplessStartHoldsTheStartingCurrent)
{
  O2PressureController c;
  c.reset(17.5);
  EXPECT_DOUBLE_EQ(c.update(1.0, default_o2_controller_params().setpoint_kpa), 17.5);
}

TEST(O2Controller, RegulatesCabinToSetpointAgainstAChangedLoad)
{
  const auto p = default_o2_controller_params();
  O2PressureController c(p);
  c.reset(17.5);
  double pp = p.setpoint_kpa - 0.3;
  const double demand_kg_day = 4.2;   // more crew load than the starting current covers
  for (int i = 0; i < 20 * 86400 / 60; ++i) {
    const double current = c.update(60.0, pp);
    pp += (current * kKpaPerSecPerAmp - demand_kg_day * kKpaPerSecPerKgDay) * 60.0;
  }
  EXPECT_NEAR(pp, p.setpoint_kpa, 0.02);
  EXPECT_NEAR(c.current(), demand_kg_day / 0.197, 0.3);
}

TEST(O2Controller, ClampsToTheStackRangeWithoutWindup)
{
  const auto p = default_o2_controller_params();
  O2PressureController c(p);
  c.reset(17.5);
  for (int i = 0; i < 1000; ++i) {
    EXPECT_LE(c.update(600.0, p.setpoint_kpa - 5.0), p.max_current_a);
  }
  // Integral is clamped, so it comes off the limit as soon as the error flips.
  EXPECT_LT(c.update(60.0, p.setpoint_kpa + 0.5), p.max_current_a);
}
