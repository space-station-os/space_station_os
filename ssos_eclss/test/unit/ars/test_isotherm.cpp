#include <gtest/gtest.h>

#include "ssos_eclss/ars/adsorption_isotherm.hpp"
#include "ssos_eclss/common/units.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::ars;

TEST(Isotherm, LoadingMonotonicInPressure)
{
  TothIsotherm iso(default_co2_on_13x());
  const double t = 298.15;
  double prev = 0.0;
  for (double torr = 0.1; torr < 10.0; torr += 0.5) {
    const double q = iso.loading(units::torr_to_pa(torr), t);
    EXPECT_GT(q, prev);
    prev = q;
  }
}

TEST(Isotherm, LoadingDecreasesWithTemperature)
{
  TothIsotherm iso(default_co2_on_13x());
  const double pp = units::torr_to_pa(3.0);
  EXPECT_GT(iso.loading(pp, 280.0), iso.loading(pp, 360.0));
}

TEST(Isotherm, ZeroPressureZeroLoading)
{
  TothIsotherm iso(default_co2_on_13x());
  EXPECT_NEAR(iso.loading(0.0, 298.15), 0.0, 1.0e-12);
}

TEST(Isotherm, AnalyticDerivativeMatchesNumeric)
{
  TothIsotherm iso(default_co2_on_13x());
  const double t = 298.15;
  const double pp = units::torr_to_pa(2.0);
  const double h = 1.0;  // Pa
  const double numeric = (iso.loading(pp + h, t) - iso.loading(pp - h, t)) / (2.0 * h);
  EXPECT_NEAR(iso.dloading_dp(pp, t), numeric, 1.0e-3 * std::abs(numeric) + 1.0e-9);
}

TEST(Isotherm, ExponentClampedToUnitInterval)
{
  TothIsotherm iso(default_co2_on_13x());
  for (double t = 200.0; t < 600.0; t += 25.0) {
    const double e = iso.exponent(t);
    EXPECT_GT(e, 0.0);
    EXPECT_LE(e, 1.0);
  }
}

TEST(Isotherm, WaterSuppressesCO2Loading)
{
  TothIsotherm co2(default_co2_on_13x());
  TothIsotherm h2o(default_h2o_on_13x());
  const double t = 298.15;
  const double pp_co2 = units::torr_to_pa(3.0);

  const auto dry = competitive_loading(co2, h2o, pp_co2, 0.0, t);
  const auto wet = competitive_loading(co2, h2o, pp_co2, units::torr_to_pa(8.0), t);

  // Water present strongly reduces CO2 capacity.
  EXPECT_LT(wet.q_co2, dry.q_co2);
  EXPECT_GT(wet.q_h2o, 0.0);
}

TEST(Isotherm, HeatOfAdsorptionPositive)
{
  TothIsotherm iso(default_co2_on_13x());
  EXPECT_GT(iso.heat_of_adsorption(), 0.0);
}
