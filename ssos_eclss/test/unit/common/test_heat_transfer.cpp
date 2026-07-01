#include <gtest/gtest.h>

#include <cmath>

#include "ssos_eclss/common/fluid_dynamics.hpp"
#include "ssos_eclss/common/heat_transfer.hpp"

using namespace ssos_eclss;

TEST(HeatTransfer, WakaoKagueiLimit)
{
  // At Re -> 0 the Nusselt number approaches 2.
  EXPECT_NEAR(heat::nusselt_wakao_kaguei(0.0, 0.7), 2.0, 1.0e-9);
  // Increases with Reynolds.
  EXPECT_GT(heat::nusselt_wakao_kaguei(100.0, 0.7),
            heat::nusselt_wakao_kaguei(10.0, 0.7));
}

TEST(HeatTransfer, EffectivenessBounds)
{
  for (double ntu = 0.1; ntu < 10.0; ntu += 0.5) {
    const double eps = heat::effectiveness_counterflow(ntu, 0.5);
    EXPECT_GE(eps, 0.0);
    EXPECT_LE(eps, 1.0);
  }
}

TEST(HeatTransfer, EffectivenessCrZeroIsExponential)
{
  const double ntu = 2.0;
  EXPECT_NEAR(heat::effectiveness_counterflow(ntu, 0.0), 1.0 - std::exp(-ntu), 1.0e-9);
}

TEST(HeatTransfer, EffectivenessMonotonicInNTU)
{
  EXPECT_GT(heat::effectiveness_counterflow(4.0, 0.5),
            heat::effectiveness_counterflow(1.0, 0.5));
}

TEST(HeatTransfer, HeatDutySign)
{
  // Hot inlet above cold inlet -> positive duty.
  EXPECT_GT(heat::hx_heat_duty(0.8, 100.0, 350.0, 300.0), 0.0);
}

TEST(HeatTransfer, SpecificSurfaceAreaPositive)
{
  const double av = heat::specific_surface_area(0.4, 0.002);
  EXPECT_GT(av, 0.0);
  // a_v = 6*(1-e)/dp = 6*0.6/0.002 = 1800.
  EXPECT_NEAR(av, 1800.0, 1.0);
}

TEST(FluidDynamics, ErgunPressureDropPositive)
{
  // Air through a packed bed.
  const double dp = fluid::ergun_pressure_drop(0.5, 0.4, 0.002, 1.2, 1.8e-5, 0.3);
  EXPECT_GT(dp, 0.0);
}

TEST(FluidDynamics, ErgunIncreasesWithVelocity)
{
  const double slow = fluid::ergun_pressure_drop(0.2, 0.4, 0.002, 1.2, 1.8e-5, 0.3);
  const double fast = fluid::ergun_pressure_drop(0.8, 0.4, 0.002, 1.2, 1.8e-5, 0.3);
  EXPECT_GT(fast, slow);
}

TEST(FluidDynamics, ReynoldsScaling)
{
  EXPECT_NEAR(fluid::reynolds_number(1.2, 1.0, 0.002, 1.8e-5),
              1.2 * 1.0 * 0.002 / 1.8e-5, 1.0e-6);
}

TEST(FluidDynamics, LaminarFrictionFactor)
{
  EXPECT_NEAR(fluid::darcy_friction_factor(1000.0), 64.0 / 1000.0, 1.0e-9);
}
