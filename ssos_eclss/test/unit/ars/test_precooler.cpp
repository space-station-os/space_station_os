#include <gtest/gtest.h>

#include "ssos_eclss/ars/precooler_model.hpp"
#include "ssos_eclss/common/units.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::ars;

namespace
{
PrecoolerParams make_params()
{
  PrecoolerParams p{};
  p.air_mass_flow = 0.015;   // kg/s
  p.coolant_mass_flow = 0.026;  // kg/s (~0.42 gpm water)
  p.air_cp = 1006.0;
  p.coolant_cp = 4180.0;
  p.ua = 30.0;  // W/K
  return p;
}
}  // namespace

TEST(Precooler, CoolsTheAir)
{
  PrecoolerModel pc(make_params());
  const double air_in = units::celsius_to_kelvin(25.0);
  const double cool_in = units::celsius_to_kelvin(7.0);
  PrecoolerResult r = pc.solve(air_in, cool_in);
  EXPECT_LT(r.air_outlet_temp, air_in);
  EXPECT_GT(r.air_outlet_temp, cool_in);
  EXPECT_GT(r.heat_duty, 0.0);
}

TEST(Precooler, ExitApproachesCoolantInletAtHighUA)
{
  PrecoolerParams p = make_params();
  p.ua = 2000.0;  // very high conductance
  PrecoolerModel pc(p);
  const double cool_in = units::celsius_to_kelvin(7.0);
  PrecoolerResult r = pc.solve(units::celsius_to_kelvin(25.0), cool_in);
  // Within ~6.5 F (~3.6 K) of the LTL water inlet (paper Fig 20).
  EXPECT_NEAR(r.air_outlet_temp, cool_in, units::fahrenheit_to_kelvin(32.0 + 6.5) -
                                          units::fahrenheit_to_kelvin(32.0));
}

TEST(Precooler, EffectivenessBounded)
{
  PrecoolerModel pc(make_params());
  PrecoolerResult r = pc.solve(300.0, 280.0);
  EXPECT_GE(r.effectiveness, 0.0);
  EXPECT_LE(r.effectiveness, 1.0);
}

TEST(Precooler, EnergyBalanceConsistent)
{
  PrecoolerParams p = make_params();
  PrecoolerModel pc(p);
  PrecoolerResult r = pc.solve(300.0, 280.0);
  const double q_air = p.air_mass_flow * p.air_cp * (300.0 - r.air_outlet_temp);
  const double q_cool = p.coolant_mass_flow * p.coolant_cp *
                        (r.coolant_outlet_temp - 280.0);
  EXPECT_NEAR(q_air, q_cool, 1.0e-6 * std::abs(q_air) + 1.0e-6);
  EXPECT_NEAR(q_air, r.heat_duty, 1.0e-6 * std::abs(q_air) + 1.0e-6);
}
