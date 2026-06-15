#include <gtest/gtest.h>

#include "ssos_eclss/common/units.hpp"
#include "ssos_eclss/sabatier/reactor_kinetics.hpp"
#include "ssos_eclss/sabatier/sabatier_system.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::sabatier;

TEST(SabatierKinetics, ConversionRisesWithTemperature)
{
  ReactorKinetics k(default_kinetics_params());
  EXPECT_LT(k.conversion(400.0), k.conversion(650.0));
  EXPECT_LE(k.conversion(700.0), default_kinetics_params().max_conversion + 1.0e-9);
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
