#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "ssos_eclss/common/gas_properties.hpp"
#include "ssos_eclss/common/units.hpp"

using namespace ssos_eclss;

TEST(GasProperties, IdealGasDensityAir)
{
  // Air at 1 atm, 273.15 K -> ~1.29 kg/m^3
  const double rho = gas::gas_density(units::STD_PRESSURE_PA, 273.15, units::M_AIR);
  EXPECT_NEAR(rho, 1.293, 0.02);
}

TEST(GasProperties, ConcentrationPartialPressureRoundTrip)
{
  const double pp = units::torr_to_pa(2.0);  // 2 torr ppCO2
  const double t = 295.0;
  const double c = gas::concentration_from_pp(pp, t);
  EXPECT_NEAR(gas::partial_pressure(c, t), pp, 1.0e-6);
}

TEST(GasProperties, SaturationPressureMonotonic)
{
  const double p_low = gas::water_saturation_pressure(280.0);
  const double p_high = gas::water_saturation_pressure(310.0);
  EXPECT_GT(p_high, p_low);
  // At 100 degC saturation pressure ~ 1 atm.
  EXPECT_NEAR(gas::water_saturation_pressure(373.15), 101325.0, 5000.0);
}

TEST(GasProperties, DewPointInvertsSaturation)
{
  const double t = 290.0;
  const double psat = gas::water_saturation_pressure(t);
  // At RH=100% dew point equals temperature.
  EXPECT_NEAR(gas::dew_point_from_pp(psat), t, 0.5);
}

TEST(GasProperties, RelativeHumidityBounds)
{
  const double t = 295.0;
  const double psat = gas::water_saturation_pressure(t);
  EXPECT_NEAR(gas::relative_humidity(0.5 * psat, t), 0.5, 1.0e-6);
  EXPECT_NEAR(gas::water_pp_from_rh(0.5, t), 0.5 * psat, 1.0e-6);
}

TEST(GasProperties, HumidityRatioPositive)
{
  const double w = gas::humidity_ratio(2000.0, units::STD_PRESSURE_PA);
  EXPECT_GT(w, 0.0);
  EXPECT_LT(w, 0.1);
}

TEST(GasProperties, MixtureMolarMassWeighting)
{
  std::vector<gas::MixtureComponent> comps = {
    {0.79, units::M_N2, 1.7e-5},
    {0.21, units::M_O2, 2.0e-5},
  };
  const double m = gas::mixture_molar_mass(comps);
  EXPECT_NEAR(m, 0.79 * units::M_N2 + 0.21 * units::M_O2, 1.0e-9);
}

TEST(GasProperties, MixtureViscosityBetweenComponents)
{
  std::vector<gas::MixtureComponent> comps = {
    {0.5, units::M_N2, 1.7e-5},
    {0.5, units::M_O2, 2.0e-5},
  };
  const double mu = gas::mixture_viscosity(comps);
  EXPECT_GT(mu, 1.6e-5);
  EXPECT_LT(mu, 2.1e-5);
}

TEST(GasProperties, SutherlandViscosityIncreasesWithT)
{
  EXPECT_GT(gas::sutherland_viscosity(350.0), gas::sutherland_viscosity(280.0));
}
