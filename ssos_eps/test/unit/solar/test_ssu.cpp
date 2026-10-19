#include <gtest/gtest.h>

#include <cmath>

#include "ssos_eps/solar/ssu_model.hpp"

using namespace ssos_eps::solar;

TEST(Ssu, RegulatesToSetpointWithConversionLoss)
{
  SsuParams p = default_ssu_params();
  p.voltage_ripple_v = 0.0;
  SsuModel ssu(p);
  const SsuOutput & out = ssu.step(1.0, 80000.0);
  EXPECT_TRUE(out.regulating);
  EXPECT_DOUBLE_EQ(out.bus_voltage_v, p.voltage_setpoint_v);
  EXPECT_NEAR(out.available_power_w, 80000.0 * p.efficiency, 1.0e-6);
  EXPECT_NEAR(out.available_current_a, out.available_power_w / p.voltage_setpoint_v, 1.0e-9);
}

TEST(Ssu, DropsOutWithoutArrayInput)
{
  SsuModel ssu;
  const SsuOutput & out = ssu.step(1.0, 0.0);
  EXPECT_FALSE(out.regulating);
  EXPECT_DOUBLE_EQ(out.bus_voltage_v, 0.0);
  EXPECT_DOUBLE_EQ(out.available_power_w, 0.0);
  EXPECT_DOUBLE_EQ(out.available_current_a, 0.0);
}

TEST(Ssu, RippleStaysWithinBand)
{
  const SsuParams p = default_ssu_params();
  SsuModel ssu(p);
  double lo = 1.0e9;
  double hi = -1.0e9;
  for (int i = 0; i < static_cast<int>(p.ripple_period_s); i += 5) {
    const double v = ssu.step(5.0, 50000.0).bus_voltage_v;
    lo = std::min(lo, v);
    hi = std::max(hi, v);
  }
  EXPECT_GE(lo, p.voltage_setpoint_v - p.voltage_ripple_v - 1.0e-9);
  EXPECT_LE(hi, p.voltage_setpoint_v + p.voltage_ripple_v + 1.0e-9);
  EXPECT_GT(hi - lo, p.voltage_ripple_v);
}
