#include <gtest/gtest.h>

#include "ssos_eps/ddcu/ddcu_model.hpp"

using namespace ssos_eps::ddcu;

namespace
{
DdcuInput supplied(double demand_w, double input_v = 160.0)
{
  return DdcuInput{input_v, demand_w, demand_w, 1.0e9};
}
}  // namespace

TEST(Ddcu, ModeNames)
{
  EXPECT_EQ(to_string(DdcuMode::OFF), "off");
  EXPECT_EQ(to_string(DdcuMode::REGULATING), "regulating");
  EXPECT_EQ(to_string(DdcuMode::OVERLOAD), "overload");
  EXPECT_EQ(to_string(DdcuMode::TRIPPED), "tripped");
}

TEST(Ddcu, RegulatesWithEfficiencyAndDroop)
{
  const DdcuParams p = default_ddcu_params();
  DdcuModel ddcu(p);
  const DdcuState & s = ddcu.step(1.0, supplied(3125.0));
  EXPECT_EQ(s.mode, DdcuMode::REGULATING);
  EXPECT_DOUBLE_EQ(s.output_power_w, 3125.0);
  EXPECT_NEAR(s.input_power_w, 3125.0 / p.efficiency + p.no_load_loss_w, 1.0e-9);
  EXPECT_NEAR(s.heat_w, s.input_power_w - s.output_power_w, 1.0e-9);
  EXPECT_NEAR(s.output_voltage_v, p.output_voltage_v - 0.5 * p.regulation_droop_v, 1.0e-9);
  EXPECT_EQ(s.status_message, "nominal");
}

TEST(Ddcu, InputWindowMatchesLegacy)
{
  DdcuModel ddcu;
  EXPECT_FALSE(ddcu.available(110.0));
  EXPECT_TRUE(ddcu.available(115.0));
  EXPECT_TRUE(ddcu.available(150.0));
  EXPECT_TRUE(ddcu.available(173.0));
  EXPECT_FALSE(ddcu.available(180.0));

  const DdcuState & s = ddcu.step(1.0, supplied(2000.0, 100.0));
  EXPECT_EQ(s.mode, DdcuMode::OFF);
  EXPECT_DOUBLE_EQ(s.output_power_w, 0.0);
  EXPECT_DOUBLE_EQ(s.output_voltage_v, 0.0);
}

TEST(Ddcu, OverloadHoldsOutputAtRating)
{
  const DdcuParams p = default_ddcu_params();
  DdcuModel ddcu(p);
  const DdcuState & s = ddcu.step(1.0, DdcuInput{160.0, 9000.0, 9000.0, 1.0e9});
  EXPECT_EQ(s.mode, DdcuMode::OVERLOAD);
  EXPECT_DOUBLE_EQ(s.output_power_w, p.rated_power_w);
  EXPECT_DOUBLE_EQ(s.load_fraction, 1.0);
}

TEST(Ddcu, BrownOutWhenPrimaryLimited)
{
  const DdcuParams p = default_ddcu_params();
  DdcuModel ddcu(p);
  const DdcuState & s = ddcu.step(1.0, DdcuInput{160.0, 4000.0, 4000.0, 2050.0});
  EXPECT_DOUBLE_EQ(s.input_power_w, 2050.0);
  EXPECT_NEAR(s.output_power_w, (2050.0 - p.no_load_loss_w) * p.efficiency, 1.0e-9);
  EXPECT_EQ(s.status_message, "primary supply limited");
}

TEST(Ddcu, FaultIsolatesOutputAndClears)
{
  DdcuModel ddcu;
  ddcu.set_fault(true);
  EXPECT_FALSE(ddcu.available(160.0));
  const DdcuState & s = ddcu.step(1.0, supplied(2000.0));
  EXPECT_EQ(s.mode, DdcuMode::TRIPPED);
  EXPECT_TRUE(s.faulted);
  EXPECT_DOUBLE_EQ(s.output_power_w, 0.0);
  ddcu.set_fault(false);
  EXPECT_EQ(ddcu.step(1.0, supplied(2000.0)).mode, DdcuMode::REGULATING);
}

TEST(Ddcu, OverTemperatureTripsAndResetsWithHysteresis)
{
  DdcuParams p = default_ddcu_params();
  p.coldplate_temp_c = 65.0;
  DdcuModel ddcu(p);
  bool tripped = false;
  for (int i = 0; i < 400 && !tripped; ++i) {
    tripped = ddcu.step(60.0, supplied(p.rated_power_w)).over_temperature;
  }
  ASSERT_TRUE(tripped);
  EXPECT_EQ(ddcu.step(1.0, supplied(p.rated_power_w)).mode, DdcuMode::TRIPPED);

  // Stays tripped until it cools below the reset temperature.
  p.coldplate_temp_c = 10.0;
  ddcu.set_params(p);
  bool reset = false;
  for (int i = 0; i < 1000 && !reset; ++i) {
    const DdcuState & s = ddcu.step(60.0, supplied(p.rated_power_w));
    if (!s.over_temperature) {
      reset = true;
      EXPECT_LE(s.temperature_c, p.reset_temp_c);
    }
  }
  EXPECT_TRUE(reset);
}

TEST(Ddcu, LossesWarmTowardEquilibrium)
{
  const DdcuParams p = default_ddcu_params();
  DdcuModel ddcu(p);
  for (int i = 0; i < 200; ++i) {
    ddcu.step(600.0, supplied(p.rated_power_w));
  }
  const DdcuState & s = ddcu.state();
  EXPECT_NEAR(s.temperature_c, p.coldplate_temp_c + s.heat_w / p.heat_transfer_w_k, 1.0e-3);
  EXPECT_LT(s.temperature_c, p.trip_temp_c);
}
