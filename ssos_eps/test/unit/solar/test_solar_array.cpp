#include <gtest/gtest.h>

#include <cmath>

#include "ssos_eps/common/units.hpp"
#include "ssos_eps/solar/solar_array_model.hpp"

using namespace ssos_eps;
using namespace ssos_eps::solar;

namespace
{
SolarArrayParams zero_beta()
{
  SolarArrayParams p = default_solar_array_params();
  p.use_fixed_beta = true;
  p.fixed_beta_deg = 0.0;
  return p;
}
}  // namespace

TEST(SolarArray, SunlitPowerIsFluxTimesAreaTimesEfficiency)
{
  const SolarArrayParams p = zero_beta();
  SolarArrayModel array(p);
  const SolarArrayState & s = array.step(1.0, 1361.0, false);
  EXPECT_TRUE(s.sunlit);
  EXPECT_DOUBLE_EQ(s.illumination, 1.0);
  EXPECT_NEAR(s.array_power_w, 1361.0 * p.area_m2 * p.cell_efficiency, 1.0e-6);
  // ISS-class peak of ~84 kW.
  EXPECT_NEAR(s.array_power_w, 84.0e3, 1.0e3);
}

TEST(SolarArray, EclipseProducesNoPower)
{
  SolarArrayModel array(zero_beta());
  const SolarArrayState & s = array.step(1.0, 1361.0, true);
  EXPECT_FALSE(s.sunlit);
  EXPECT_DOUBLE_EQ(s.array_power_w, 0.0);
  EXPECT_DOUBLE_EQ(s.illumination, 0.0);
}

TEST(SolarArray, SunriseRampIsMonotonicAndReachesFullOutput)
{
  const SolarArrayParams p = zero_beta();
  SolarArrayModel array(p);
  array.step(1.0, 0.0, true);
  double last = -1.0;
  for (int i = 0; i < static_cast<int>(p.sunrise_ramp_s); i += 10) {
    const SolarArrayState & s = array.step(10.0, 1361.0, false);
    EXPECT_GE(s.array_power_w, last);
    EXPECT_LE(s.illumination, 1.0);
    last = s.array_power_w;
  }
  EXPECT_NEAR(array.state().illumination, 1.0, 1.0e-9);
  EXPECT_EQ(array.state().orbit_count, 1);
}

TEST(SolarArray, IlluminationIsHalfwayAtMidRamp)
{
  const SolarArrayParams p = zero_beta();
  SolarArrayModel array(p);
  array.step(1.0, 0.0, true);
  array.step(0.5 * p.sunrise_ramp_s, 1361.0, false);
  EXPECT_NEAR(array.state().illumination, 0.5, 1.0e-9);
}

TEST(SolarArray, BetaCosineDerate)
{
  SolarArrayParams p = zero_beta();
  p.fixed_beta_deg = 60.0;
  SolarArrayModel array(p);
  const SolarArrayState & s = array.step(1.0, 1361.0, false);
  EXPECT_FALSE(s.gimbal_locked);
  EXPECT_NEAR(s.beta_derate, 0.5, 1.0e-9);
  EXPECT_NEAR(s.array_power_w, 0.5 * 1361.0 * p.area_m2 * p.cell_efficiency, 1.0e-6);
}

TEST(SolarArray, GimbalLockAppliesExtraDerate)
{
  SolarArrayParams p = zero_beta();
  p.fixed_beta_deg = -75.0;
  SolarArrayModel array(p);
  const SolarArrayState & s = array.step(1.0, 1361.0, false);
  EXPECT_TRUE(s.gimbal_locked);
  EXPECT_NEAR(s.beta_derate, std::cos(units::deg_to_rad(75.0)) * p.gimbal_lock_derate, 1.0e-9);
}

TEST(SolarArray, BetaPrecessesOncePerCycleOfSunrises)
{
  SolarArrayParams p = default_solar_array_params();
  p.beta_max_deg = 40.0;
  p.beta_precession_orbits = 4;
  SolarArrayModel array(p);
  array.step(1.0, 1361.0, false);
  EXPECT_NEAR(array.state().beta_deg, 0.0, 1.0e-9);

  array.step(1.0, 0.0, true);
  array.step(1.0, 1361.0, false);
  EXPECT_NEAR(array.state().beta_deg, 40.0, 1.0e-9);

  for (int k = 0; k < 3; ++k) {
    array.step(1.0, 0.0, true);
    array.step(1.0, 1361.0, false);
  }
  EXPECT_EQ(array.state().orbit_count, 4);
  EXPECT_NEAR(array.state().beta_deg, 0.0, 1.0e-9);
}

TEST(SolarArray, DegradationScalesOutput)
{
  SolarArrayParams p = zero_beta();
  SolarArrayModel nominal(p);
  p.degradation_factor = 0.4;
  SolarArrayModel degraded(p);
  const double full = nominal.step(1.0, 1361.0, false).array_power_w;
  const double part = degraded.step(1.0, 1361.0, false).array_power_w;
  EXPECT_NEAR(part, 0.4 * full, 1.0e-6);
}

TEST(SolarArray, PanelTemperatureFollowsFirstOrderLag)
{
  const SolarArrayParams p = zero_beta();
  const PanelThermalParams th = default_panel_thermal_params();
  SolarArrayModel array(p, th);
  const double power_kw = array.step(0.0, 1361.0, false).array_power_w / 1000.0;
  const double target = th.ambient_temp_c + th.temp_rise_per_kw_c * power_kw;

  array.step(th.time_constant_s, 1361.0, false);
  const double expected = th.ambient_temp_c +
    (target - th.ambient_temp_c) * (1.0 - std::exp(-1.0));
  EXPECT_NEAR(array.state().panel_temperature_c, expected, 1.0e-6);

  for (int i = 0; i < 20; ++i) {
    array.step(th.time_constant_s, 1361.0, false);
  }
  EXPECT_NEAR(array.state().panel_temperature_c, target, 1.0e-3);

  for (int i = 0; i < 20; ++i) {
    array.step(th.time_constant_s, 0.0, true);
  }
  EXPECT_NEAR(array.state().panel_temperature_c, th.ambient_temp_c, 1.0e-3);
}
