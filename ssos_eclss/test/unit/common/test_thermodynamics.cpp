#include <gtest/gtest.h>

#include "ssos_eclss/common/thermodynamics.hpp"

using namespace ssos_eclss;
using thermo::Species;

TEST(Thermodynamics, AirCpReasonable)
{
  // cp of air near room temp ~1005 J/(kg*K).
  const double cp = thermo::cp_mass(Species::AIR, 300.0);
  EXPECT_NEAR(cp, 1005.0, 60.0);
}

TEST(Thermodynamics, GammaAirNear14)
{
  EXPECT_NEAR(thermo::gamma_ratio(Species::AIR, 300.0), 1.4, 0.05);
}

TEST(Thermodynamics, CvLessThanCp)
{
  EXPECT_LT(thermo::cv_mass(Species::CO2, 300.0), thermo::cp_mass(Species::CO2, 300.0));
}

TEST(Thermodynamics, SensibleEnthalpyZeroAtReference)
{
  EXPECT_NEAR(thermo::sensible_enthalpy(Species::AIR, 298.15, 298.15), 0.0, 1.0e-6);
}

TEST(Thermodynamics, SensibleEnthalpyIncreasesWithT)
{
  const double h1 = thermo::sensible_enthalpy(Species::AIR, 350.0);
  const double h0 = thermo::sensible_enthalpy(Species::AIR, 300.0);
  EXPECT_GT(h1, h0);
  // Roughly cp * dT.
  EXPECT_NEAR(h1 - h0, 1005.0 * 50.0, 0.2 * 1005.0 * 50.0);
}

TEST(Thermodynamics, LatentHeatWaterDecreasesWithT)
{
  const double l_cold = thermo::latent_heat_water(280.0);
  const double l_hot = thermo::latent_heat_water(360.0);
  EXPECT_GT(l_cold, l_hot);
  // ~2.45e6 J/kg near room temperature.
  EXPECT_NEAR(thermo::latent_heat_water(298.15), 2.44e6, 1.0e5);
}

TEST(Thermodynamics, LiquidWaterCpNear4180)
{
  EXPECT_NEAR(thermo::cp_liquid_water(300.0), 4180.0, 50.0);
}

TEST(Thermodynamics, SorbentCpPositive)
{
  EXPECT_GT(thermo::cp_sorbent_solid(400.0), 0.0);
}
