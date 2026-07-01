#include <gtest/gtest.h>

#include "ssos_eclss/common/units.hpp"
#include "ssos_eclss/sabatier/reactor_kinetics.hpp"
#include "ssos_eclss/sabatier/sabatier_system.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::sabatier;

TEST(SabatierKinetics, KineticLimitedAtLowTemperature)
{
  // Below ~375 C (648 K) kinetics are slow, so conversion is well below the
  // equilibrium ceiling.
  ReactorKinetics k(default_kinetics_params());
  EXPECT_LT(k.conversion(500.0), 0.7);
  EXPECT_LT(k.conversion(500.0), k.conversion(648.0));
}

TEST(SabatierKinetics, ISSOperatingConversionAbout95Percent)
{
  // ICES-2018-155 / ISS Sabatier: ~95% CO2 conversion at the operating point.
  ReactorKinetics k(default_kinetics_params());
  EXPECT_NEAR(k.conversion(648.0), 0.95, 0.03);
}

TEST(SabatierKinetics, EquilibriumLimitedAtHighTemperature)
{
  // The exothermic equilibrium conversion declines at high temperature, so
  // conversion peaks at an intermediate T rather than rising monotonically.
  ReactorKinetics k(default_kinetics_params());
  EXPECT_LT(k.conversion(800.0), k.conversion(648.0));
  EXPECT_LE(k.conversion(900.0), default_kinetics_params().max_conversion + 1.0e-9);
}

TEST(SabatierSystem, StoichiometryAndProducts)
{
  SabatierSystem s;
  s.reset(600.0);
  // Ample H2 (excess), CO2-limited.
  const double co2 = 1.0e-4;
  const double h2 = 8.0e-4;  // > 4*co2
  const SabatierResult r = s.step(1.0, co2, h2);
  EXPECT_GT(r.co2_consumed_mol_s, 0.0);
  EXPECT_NEAR(r.h2_consumed_mol_s, 4.0 * r.co2_consumed_mol_s, 1.0e-12);
  EXPECT_NEAR(r.ch4_produced_mol_s, r.co2_consumed_mol_s, 1.0e-12);
  EXPECT_NEAR(r.water_produced_mol_s, 2.0 * r.co2_consumed_mol_s, 1.0e-12);
  EXPECT_FALSE(r.hydrogen_limited);
}

TEST(SabatierSystem, HydrogenLimited)
{
  SabatierSystem s;
  s.reset(600.0);
  // Insufficient H2.
  const SabatierResult r = s.step(1.0, 1.0e-3, 1.0e-4);
  EXPECT_TRUE(r.hydrogen_limited);
}

TEST(SabatierSystem, ExothermicHeatsReactor)
{
  // Started warm, the exotherm sustains/raises the reactor temperature against
  // heat loss (the reactor self-sustains once lit).
  SabatierSystem s;
  s.reset(550.0);
  SabatierResult r{};
  for (int i = 0; i < 200; ++i) {
    r = s.step(1.0, 5.0e-3, 4.0e-2);  // generous feed
  }
  EXPECT_GT(r.reaction_heat_w, 0.0);
  EXPECT_GT(r.reactor_temp_k, 550.0);
}

TEST(SabatierSystem, HigherFeedRaisesSteadyTemperature)
{
  auto steady_temp = [](double co2, double h2) {
    SabatierSystem s;
    s.reset(600.0);
    SabatierResult r{};
    for (int i = 0; i < 2000; ++i) {
      r = s.step(1.0, co2, h2);
    }
    return r.reactor_temp_k;
  };
  EXPECT_GT(steady_temp(2.0e-3, 1.6e-2), steady_temp(5.0e-4, 4.0e-3));
}

TEST(SabatierSystem, ProducesWaterForWrs)
{
  SabatierSystem s;
  s.reset(600.0);
  const SabatierResult r = s.step(1.0, 1.0e-4, 8.0e-4);
  EXPECT_GT(r.water_produced_kg_s, 0.0);
  EXPECT_NEAR(r.water_produced_kg_s, r.water_produced_mol_s * units::M_H2O, 1.0e-15);
}

// Regression: at flight CO2 rates the exotherm alone is below the heat loss, so
// without the thermostatic trim heater the reactor cooled and conversion
// collapsed to zero. The heater must hold the setpoint and keep conversion high.
TEST(SabatierSystem, TrimHeaterHoldsTemperatureAtFlightRate)
{
  SabatierSystem s;  // starts at the operating setpoint (~648 K)
  const double co2 = 4.16 / 86400.0 / units::M_CO2;  // ~4.16 kg/day
  const double h2 = 2.0 * (5.3 / 86400.0 / units::M_O2);  // OGS H2 (2 per O2)
  SabatierResult r{};
  for (int i = 0; i < 6000; ++i) {
    r = s.step(1.0, co2, h2);
  }
  EXPECT_NEAR(r.reactor_temp_k, 648.0, 20.0);  // held near setpoint (no collapse)
  EXPECT_GT(r.conversion, 0.9);
  EXPECT_GT(r.heater_power_w, 0.0);            // heater supplies the deficit
}

TEST(SabatierSystem, TrimHeaterRecoversFromCoolStart)
{
  SabatierSystem s;
  s.reset(500.0);  // below the kinetic sweet spot
  const double co2 = 4.16 / 86400.0 / units::M_CO2;
  const double h2 = 2.0 * (5.3 / 86400.0 / units::M_O2);
  SabatierResult r{};
  for (int i = 0; i < 6000; ++i) {
    r = s.step(1.0, co2, h2);
  }
  EXPECT_GT(r.reactor_temp_k, 600.0);  // pulled back up, not collapsed
  EXPECT_GT(r.conversion, 0.8);
}
