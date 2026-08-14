#include <gtest/gtest.h>

#include "ssos_eclss/ars/blower_model.hpp"
#include "ssos_eclss/common/units.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::ars;

namespace
{
BlowerParams make_params(double q_design)
{
  BlowerParams p{};
  p.rated_rpm = 12000.0;
  p.shutoff_dp = 2.0 * units::in_h2o_to_pa(40.0);
  p.max_rpm = 18000.0;
  p.curve_quad = units::in_h2o_to_pa(40.0) / (q_design * q_design);
  return p;
}
}  // namespace

TEST(Blower, ConstantFlowDeliversSetpoint)
{
  const double q_design = units::scfm_to_m3s(26.0);
  BlowerModel b(make_params(q_design));
  // System resistance giving ~40 in-H2O at design flow.
  const double r = units::in_h2o_to_pa(40.0) / (q_design * q_design);
  BlowerResult res = b.solve(r, BlowerControl::CONSTANT_FLOW, q_design);
  EXPECT_NEAR(res.flow_m3s, q_design, 1.0e-9);
  EXPECT_GT(res.delta_p, 0.0);
}

TEST(Blower, OperatingHeadInExpectedRange)
{
  const double q_design = units::scfm_to_m3s(26.0);
  BlowerModel b(make_params(q_design));
  const double r = units::in_h2o_to_pa(38.0) / (q_design * q_design);
  BlowerResult res = b.solve(r, BlowerControl::CONSTANT_RPM, 12000.0);
  const double head_in_h2o = units::pa_to_in_h2o(res.delta_p);
  // Target ~37-40 in-H2O.
  EXPECT_GT(head_in_h2o, 15.0);
  EXPECT_LT(head_in_h2o, 60.0);
}

TEST(Blower, HigherResistanceLowersFlow)
{
  const double q_design = units::scfm_to_m3s(26.0);
  BlowerModel b(make_params(q_design));
  const double r_low = units::in_h2o_to_pa(20.0) / (q_design * q_design);
  const double r_high = units::in_h2o_to_pa(80.0) / (q_design * q_design);
  BlowerResult low = b.solve(r_low, BlowerControl::CONSTANT_RPM, 12000.0);
  BlowerResult high = b.solve(r_high, BlowerControl::CONSTANT_RPM, 12000.0);
  EXPECT_GT(low.flow_m3s, high.flow_m3s);
}

TEST(Blower, PowerPositive)
{
  const double q_design = units::scfm_to_m3s(26.0);
  BlowerModel b(make_params(q_design));
  const double r = units::in_h2o_to_pa(40.0) / (q_design * q_design);
  BlowerResult res = b.solve(r, BlowerControl::CONSTANT_FLOW, q_design);
  EXPECT_GT(res.power_w, 0.0);
}
