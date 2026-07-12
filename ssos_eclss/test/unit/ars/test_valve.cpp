#include <gtest/gtest.h>

#include "ssos_eclss/ars/valve_model.hpp"
#include "ssos_eclss/common/units.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::ars;

namespace
{
ValveModel make_valve()
{
  ValveParams p{};
  p.cv = 1.0e-4;
  p.stroke_time_s = 2.0;
  p.repress_time_s = 5.0;
  return ValveModel(p);
}
}  // namespace

TEST(Valve, SlewsTowardCommand)
{
  ValveModel v = make_valve();
  v.command(1.0);
  EXPECT_NEAR(v.position(), 0.0, 1.0e-9);
  for (int i = 0; i < 5; ++i) {
    v.update(0.5);
  }
  EXPECT_NEAR(v.position(), 1.0, 1.0e-9);
}

TEST(Valve, FlowScalesWithPosition)
{
  ValveModel v = make_valve();
  v.command(1.0);
  v.update(10.0);  // fully open
  const double q_open = v.flow(1000.0);
  EXPECT_GT(q_open, 0.0);
  // Closed valve passes nothing.
  ValveModel closed = make_valve();
  EXPECT_NEAR(closed.flow(1000.0), 0.0, 1.0e-12);
}

TEST(Valve, FlowReversesWithPressure)
{
  ValveModel v = make_valve();
  v.command(1.0);
  v.update(10.0);
  EXPECT_GT(v.flow(1000.0), 0.0);
  EXPECT_LT(v.flow(-1000.0), 0.0);
}

TEST(Valve, RepressurizationReaches800Torr)
{
  ValveModel v = make_valve();
  const double target = units::torr_to_pa(800.0);
  double p = 0.0;
  // ~3 time constants (15 s) should fill to >95%.
  for (int i = 0; i < 30; ++i) {
    p = v.repressurize(0.5, p, target);
  }
  EXPECT_GT(p, 0.95 * target);
  EXPECT_LE(p, target + 1.0);
}
