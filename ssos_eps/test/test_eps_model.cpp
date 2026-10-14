#include <gtest/gtest.h>

#include "ssos_eps/eps_model.hpp"

namespace
{

TEST(EpsModelTest, GeneratesPowerInSunlight)
{
  ssos_eps::EpsConfig config;
  config.solar_array_area_m2 = 100.0;
  config.solar_array_efficiency = 0.30;
  config.load_demand_w = 20000.0;

  ssos_eps::EpsModel model(config);

  ssos_eps::EpsInput input;
  input.solar_flux_w_m2 = 1361.0;
  input.in_eclipse = false;
  input.dt_s = 1.0;

  const auto state = model.update(input);

  EXPECT_NEAR(state.generation_w, 40830.0, 1.0);
  EXPECT_GT(state.power_balance_w, 0.0);
  EXPECT_TRUE(state.healthy);
}

TEST(EpsModelTest, ProducesNoSolarPowerInEclipse)
{
  ssos_eps::EpsModel model;

  ssos_eps::EpsInput input;
  input.solar_flux_w_m2 = 1361.0;
  input.in_eclipse = true;
  input.dt_s = 1.0;

  const auto state = model.update(input);

  EXPECT_DOUBLE_EQ(state.generation_w, 0.0);
  EXPECT_LT(state.power_balance_w, 0.0);
}

TEST(EpsModelTest, BatteryDischargesDuringPowerDeficit)
{
  ssos_eps::EpsConfig config;
  config.load_demand_w = 10000.0;
  config.battery_capacity_wh = 100000.0;
  config.initial_battery_soc = 0.50;

  ssos_eps::EpsModel model(config);

  ssos_eps::EpsInput input;
  input.in_eclipse = true;
  input.dt_s = 3600.0;

  const auto state = model.update(input);

  EXPECT_NEAR(state.battery_soc, 0.40, 1.0e-9);
}

TEST(EpsModelTest, ReportsDegradedHealthAtLowBatterySoc)
{
  ssos_eps::EpsConfig config;
  config.initial_battery_soc = 0.15;
  config.low_soc_threshold = 0.20;

  ssos_eps::EpsModel model(config);

  ssos_eps::EpsInput input;
  input.in_eclipse = true;
  input.dt_s = 1.0;

  const auto state = model.update(input);

  EXPECT_FALSE(state.healthy);
}

TEST(EpsModelTest, BatterySocIsClampedToValidRange)
{
  ssos_eps::EpsConfig config;
  config.load_demand_w = 100000.0;
  config.battery_capacity_wh = 1000.0;
  config.initial_battery_soc = 0.10;

  ssos_eps::EpsModel model(config);

  ssos_eps::EpsInput input;
  input.in_eclipse = true;
  input.dt_s = 3600.0;

  const auto state = model.update(input);

  EXPECT_DOUBLE_EQ(state.battery_soc, 0.0);
}

}  // namespace
