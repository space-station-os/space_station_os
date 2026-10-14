#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "ssos_eps/battery/battery_oru_model.hpp"
#include "ssos_eps/bcdu/bcdu_model.hpp"

using namespace ssos_eps;
using namespace ssos_eps::bcdu;
using battery::BatteryOruModel;

namespace
{
std::vector<BatteryOruModel> two_orus(double soc)
{
  std::vector<BatteryOruModel> orus(2, BatteryOruModel());
  for (auto & o : orus) {
    o.reset(soc, 10.0);
  }
  return orus;
}

BcduInput sunlit(double source_w, double load_w)
{
  return BcduInput{160.0, source_w, load_w};
}

BcduInput eclipse(double load_w)
{
  return BcduInput{0.0, 0.0, load_w};
}

// Bus-side power balance: source + battery discharge = load + charge + shunt.
void expect_balance(const BcduInput & in, const BcduState & s)
{
  const double source = s.source_available ? in.source_power_w : 0.0;
  const double into_bus = source + std::max(s.bus_power_w, 0.0);
  const double out_of_bus = s.load_served_w + std::max(-s.bus_power_w, 0.0) +
    s.shunted_power_w;
  EXPECT_NEAR(into_bus, out_of_bus, 1.0e-6);
}
}  // namespace

TEST(Bcdu, ModeNames)
{
  EXPECT_EQ(to_string(BcduMode::IDLE), "idle");
  EXPECT_EQ(to_string(BcduMode::CHARGING), "charging");
  EXPECT_EQ(to_string(BcduMode::DISCHARGING), "discharging");
  EXPECT_EQ(to_string(BcduMode::FAULT), "fault");
}

TEST(Bcdu, ChargesFromSurplusInSunlight)
{
  BcduModel bcdu;
  auto orus = two_orus(0.6);
  const BcduInput in = sunlit(7000.0, 2500.0);
  const BcduState & s = bcdu.step(1.0, in, orus);
  EXPECT_TRUE(s.source_available);
  EXPECT_EQ(s.mode, BcduMode::CHARGING);
  EXPECT_LT(s.battery_power_w, 0.0);
  EXPECT_LT(s.current_draw_a, 0.0);
  EXPECT_DOUBLE_EQ(s.load_served_w, 2500.0);
  EXPECT_DOUBLE_EQ(s.unserved_load_w, 0.0);
  EXPECT_NEAR(s.bus_voltage_v, 160.0, 1.0e-9);
  EXPECT_NEAR(s.battery_power_w, s.bus_power_w * bcdu.params().charge_efficiency, 1.0e-6);
  EXPECT_FALSE(s.degraded);
  expect_balance(in, s);
  for (const auto & o : orus) {
    EXPECT_EQ(o.state().mode, battery::BatteryMode::CHARGING);
  }
}

TEST(Bcdu, DischargesToCarryLoadInEclipse)
{
  BcduModel bcdu;
  auto orus = two_orus(0.8);
  const BcduInput in = eclipse(2500.0);
  const BcduState & s = bcdu.step(1.0, in, orus);
  EXPECT_FALSE(s.source_available);
  EXPECT_EQ(s.mode, BcduMode::DISCHARGING);
  EXPECT_NEAR(s.load_served_w, 2500.0, 1.0e-6);
  EXPECT_NEAR(s.bus_voltage_v, bcdu.params().regulation_voltage_v, 1.0e-9);
  EXPECT_NEAR(s.current_draw_a, 2500.0 / bcdu.params().regulation_voltage_v, 1.0e-6);
  EXPECT_NEAR(s.battery_power_w * bcdu.params().discharge_efficiency, 2500.0, 1.0e-6);
  EXPECT_GT(s.conversion_loss_w, 0.0);
  expect_balance(in, s);
}

TEST(Bcdu, DischargesToCoverDeficitInSunlight)
{
  BcduModel bcdu;
  auto orus = two_orus(0.8);
  const BcduInput in = sunlit(1000.0, 3000.0);
  const BcduState & s = bcdu.step(1.0, in, orus);
  EXPECT_EQ(s.mode, BcduMode::DISCHARGING);
  EXPECT_NEAR(s.bus_power_w, 2000.0, 1.0e-6);
  EXPECT_NEAR(s.load_served_w, 3000.0, 1.0e-6);
  EXPECT_DOUBLE_EQ(s.shunted_power_w, 0.0);
  expect_balance(in, s);
}

TEST(Bcdu, HysteresisHoldsSourceStateBetweenThresholds)
{
  BcduModel bcdu;
  auto orus = two_orus(0.8);
  bcdu.step(1.0, BcduInput{160.0, 5000.0, 1000.0}, orus);
  EXPECT_TRUE(bcdu.state().source_available);
  bcdu.step(1.0, BcduInput{155.0, 5000.0, 1000.0}, orus);
  EXPECT_TRUE(bcdu.state().source_available);
  bcdu.step(1.0, BcduInput{150.0, 5000.0, 1000.0}, orus);
  EXPECT_FALSE(bcdu.state().source_available);
  EXPECT_EQ(bcdu.state().mode, BcduMode::DISCHARGING);
  bcdu.step(1.0, BcduInput{155.0, 5000.0, 1000.0}, orus);
  EXPECT_FALSE(bcdu.state().source_available);
}

TEST(Bcdu, IdlesAndShuntsWhenBatteriesFull)
{
  BcduModel bcdu;
  auto orus = two_orus(1.0);
  const BcduInput in = sunlit(7000.0, 2500.0);
  const BcduState & s = bcdu.step(1.0, in, orus);
  EXPECT_EQ(s.mode, BcduMode::IDLE);
  EXPECT_DOUBLE_EQ(s.battery_power_w, 0.0);
  EXPECT_NEAR(s.shunted_power_w, 4500.0, 1.0e-6);
  expect_balance(in, s);
}

TEST(Bcdu, ChargeLimitedByBusCurrent)
{
  BcduParams p = default_bcdu_params();
  p.max_charge_current_a = 10.0;
  BcduModel bcdu(p);
  auto orus = two_orus(0.5);
  const BcduInput in = sunlit(20000.0, 0.0);
  const BcduState & s = bcdu.step(1.0, in, orus);
  EXPECT_NEAR(-s.bus_power_w, 10.0 * 160.0, 1.0e-6);
  EXPECT_NEAR(s.shunted_power_w, 20000.0 - 1600.0, 1.0e-6);
  expect_balance(in, s);
}

TEST(Bcdu, FaultIsolatesBatteries)
{
  BcduModel bcdu;
  auto orus = two_orus(0.8);
  bcdu.set_fault(true);
  const BcduInput in = eclipse(2500.0);
  const BcduState & s = bcdu.step(1.0, in, orus);
  EXPECT_EQ(s.mode, BcduMode::FAULT);
  EXPECT_TRUE(s.faulted);
  EXPECT_DOUBLE_EQ(s.battery_power_w, 0.0);
  EXPECT_DOUBLE_EQ(s.unserved_load_w, 2500.0);
  EXPECT_DOUBLE_EQ(s.bus_voltage_v, 0.0);
  EXPECT_TRUE(s.degraded);
  for (const auto & o : orus) {
    EXPECT_DOUBLE_EQ(o.state().current_a, 0.0);
  }

  bcdu.set_fault(false);
  EXPECT_EQ(bcdu.step(1.0, in, orus).mode, BcduMode::DISCHARGING);
}

TEST(Bcdu, FailedOruDegradesChannelButRemainingOruCarriesLoad)
{
  BcduModel bcdu;
  auto orus = two_orus(0.8);
  orus[0].set_failed(true);
  const BcduInput in = eclipse(2500.0);
  const BcduState & s = bcdu.step(1.0, in, orus);
  EXPECT_EQ(s.mode, BcduMode::DISCHARGING);
  EXPECT_TRUE(s.degraded);
  EXPECT_EQ(s.failed_orus, 1);
  EXPECT_NEAR(s.load_served_w, 2500.0, 1.0e-6);
  EXPECT_DOUBLE_EQ(orus[0].state().current_a, 0.0);
  EXPECT_GT(orus[1].state().current_a, 0.0);
}

TEST(Bcdu, EmptyBatteriesLeaveLoadUnserved)
{
  BcduModel bcdu;
  auto orus = two_orus(0.0);
  const BcduInput in = eclipse(2500.0);
  const BcduState & s = bcdu.step(1.0, in, orus);
  EXPECT_EQ(s.mode, BcduMode::IDLE);
  EXPECT_DOUBLE_EQ(s.unserved_load_w, 2500.0);
  EXPECT_TRUE(s.degraded);
  EXPECT_DOUBLE_EQ(s.bus_voltage_v, 0.0);
}

TEST(Bcdu, OrusShareLoadEvenlyWhenMatched)
{
  BcduModel bcdu;
  auto orus = two_orus(0.7);
  bcdu.step(1.0, eclipse(3000.0), orus);
  EXPECT_NEAR(orus[0].state().current_a, orus[1].state().current_a, 1.0e-9);
}
