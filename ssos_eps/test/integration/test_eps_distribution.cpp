#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "ssos_eps/system/eps_system.hpp"

using namespace ssos_eps;
using namespace ssos_eps::system;

namespace
{
const EpsInput kSun{1361.0, false};
const EpsInput kShadow{0.0, true};

EpsParameters zero_beta()
{
  EpsParameters p = default_eps_parameters();
  p.solar.array.use_fixed_beta = true;
  p.solar.array.fixed_beta_deg = 0.0;
  return p;
}

const loads::LoadState * find_load(const EpsState & s, const std::string & id)
{
  for (const auto & l : s.loads) {
    if (l.id == id) {
      return &l;
    }
  }
  return nullptr;
}
}  // namespace

TEST(EpsDistribution, DefaultTopology)
{
  EpsSystem eps;
  EXPECT_EQ(eps.n_mbsu(), 4);
  EXPECT_EQ(eps.n_ddcu(), 8);
  EXPECT_EQ(eps.loads().loads().size(), 10u);
  EXPECT_NEAR(eps.loads().total_requested_w(), 30000.0, 1.0e-9);
  for (int d = 0; d < eps.n_ddcu(); ++d) {
    EXPECT_LE(eps.loads().requested_on_ddcu(d), eps.parameters().ddcu.rated_power_w);
  }
}

TEST(EpsDistribution, NominalLoadsAreAllServed)
{
  EpsSystem eps(zero_beta());
  const EpsState & s = eps.step(1.0, kSun);
  EXPECT_NEAR(s.load_demand_w, 30000.0, 1.0e-6);
  EXPECT_NEAR(s.load_served_w, 30000.0, 1.0e-6);
  EXPECT_EQ(s.shed_loads, 0);
  EXPECT_TRUE(s.healthy);
  for (const auto & m : s.mbsus) {
    EXPECT_FALSE(m.unstable);
    EXPECT_EQ(m.available_sources, 3);
  }
  for (const auto & d : s.ddcus) {
    EXPECT_EQ(d.mode, ddcu::DdcuMode::REGULATING);
    EXPECT_GT(d.output_voltage_v, 123.0);
  }
}

TEST(EpsDistribution, LoadDemandDrivesChannelsAndBatteries)
{
  EpsSystem eps(zero_beta());
  const EpsState & s = eps.step(1.0, kShadow);
  const auto & p = eps.parameters();
  // Primary draw = load / DDCU efficiency + standby losses of every DDCU.
  const double expected_primary = 30000.0 / p.ddcu.efficiency +
    eps.n_ddcu() * p.ddcu.no_load_loss_w;
  EXPECT_NEAR(s.primary_load_w, expected_primary, 1.0e-3);
  EXPECT_NEAR(s.primary_load_w, s.load_served_w + s.distribution_loss_w, 1.0e-3);
  EXPECT_NEAR(s.battery_discharge_w, s.primary_load_w / p.bcdu.discharge_efficiency, 1.0e-3);
  EXPECT_NEAR(s.power_balance_w, -s.primary_load_w, 1.0e-6);
}

TEST(EpsDistribution, RequestedLoadIsAdmittedAndServed)
{
  EpsSystem eps(zero_beta());
  const double primary_before = eps.step(1.0, kSun).primary_load_w;

  const LoadResult r = eps.request_load("experiment", 2000.0, 2);
  ASSERT_TRUE(r.accepted) << r.message;
  EXPECT_GE(r.ddcu, 0);
  EXPECT_DOUBLE_EQ(r.granted_w, 2000.0);
  EXPECT_GT(r.supply_voltage_v, 123.0);

  const EpsState & s = eps.step(1.0, kSun);
  const auto * load = find_load(s, "experiment");
  ASSERT_NE(load, nullptr);
  EXPECT_DOUBLE_EQ(load->served_w, 2000.0);
  EXPECT_NEAR(s.load_demand_w, 32000.0, 1.0e-6);
  EXPECT_NEAR(s.primary_load_w - primary_before, 2000.0 / eps.parameters().ddcu.efficiency,
              1.0e-3);
}

TEST(EpsDistribution, MultipleLoadsStayWithinDdcuRatings)
{
  EpsSystem eps(zero_beta());
  eps.step(1.0, kSun);
  for (int i = 0; i < 5; ++i) {
    const LoadResult r = eps.request_load("rack_" + std::to_string(i), 2500.0, 3);
    ASSERT_TRUE(r.accepted) << r.message;
    eps.step(1.0, kSun);
  }
  for (int d = 0; d < eps.n_ddcu(); ++d) {
    EXPECT_LE(eps.loads().requested_on_ddcu(d), eps.parameters().ddcu.rated_power_w + 1.0e-6);
  }
  const EpsState & s = eps.state();
  EXPECT_NEAR(s.load_served_w, 42500.0, 1.0e-6);
  EXPECT_EQ(s.shed_loads, 0);

  // The bus is now close to full: one more rack does not fit any DDCU.
  const LoadResult full = eps.request_load("rack_extra", 2500.0, 3);
  EXPECT_FALSE(full.accepted);
}

TEST(EpsDistribution, UpdatingALoadReplacesIt)
{
  EpsSystem eps(zero_beta());
  eps.step(1.0, kSun);
  ASSERT_TRUE(eps.request_load("heater", 1000.0, 2).accepted);
  ASSERT_TRUE(eps.request_load("heater", 1500.0, 1).accepted);
  EXPECT_EQ(eps.loads().loads().size(), 11u);
  EXPECT_DOUBLE_EQ(eps.loads().find("heater")->power_w, 1500.0);
}

TEST(EpsDistribution, InvalidRequestsAreRejected)
{
  EpsSystem eps(zero_beta());
  eps.step(1.0, kSun);
  EXPECT_FALSE(eps.request_load("", 100.0, 1).accepted);
  EXPECT_FALSE(eps.request_load("x", 0.0, 1).accepted);
  EXPECT_FALSE(eps.request_load("x", -50.0, 1).accepted);
  EXPECT_FALSE(eps.request_load("x", std::nan(""), 1).accepted);
  EXPECT_FALSE(eps.request_load("x", std::numeric_limits<double>::infinity(), 1).accepted);
  EXPECT_FALSE(eps.request_load("x", 100.0, -1).accepted);
  EXPECT_FALSE(eps.request_load("x", 100.0, 1, 99).accepted);
  EXPECT_FALSE(eps.request_load("x", 100.0, 1, -2).accepted);
  // Larger than any single DDCU rating.
  EXPECT_FALSE(eps.request_load("x", 7000.0, 1).accepted);
  EXPECT_FALSE(eps.release_load("never_requested").accepted);
  EXPECT_EQ(eps.loads().find("x"), nullptr);
  EXPECT_NEAR(eps.loads().total_requested_w(), 30000.0, 1.0e-9);
}

TEST(EpsDistribution, RequestBeyondSupplyMarginIsRejected)
{
  EpsParameters p = zero_beta();
  // 24 A at 150 V per channel: 10.8 kW per MBSU in eclipse. MBSU 1 (DDCUs 2
  // and 3) carries the most load and is left with only a few hundred watts.
  p.bcdu.max_discharge_current_a = 24.0;
  EpsSystem eps(p);
  const EpsState & s = eps.step(1.0, kShadow);
  ASSERT_EQ(s.shed_loads, 0);
  const double margin = s.mbsus[1].capability_w - s.mbsus[1].supplied_w;
  ASSERT_GT(margin, 0.0);
  ASSERT_LT(margin, 1000.0);

  // DDCU 2 has rating headroom, but its MBSU cannot supply 2 kW more.
  const LoadResult on_mbsu1 = eps.request_load("big", 2000.0, 2, 2);
  EXPECT_FALSE(on_mbsu1.accepted);
  EXPECT_NE(on_mbsu1.message.find("margin"), std::string::npos);

  // Letting EPS choose places it behind an MBSU with spare supply.
  const LoadResult anywhere = eps.request_load("big", 2000.0, 2);
  ASSERT_TRUE(anywhere.accepted) << anywhere.message;
  EXPECT_NE(p.ddcu_mbsu[static_cast<std::size_t>(anywhere.ddcu)], 1);
}

TEST(EpsDistribution, ReleasedLoadReducesDemand)
{
  EpsSystem eps(zero_beta());
  eps.step(1.0, kSun);
  ASSERT_TRUE(eps.release_load("lab_payloads_a").accepted);
  const EpsState & s = eps.step(1.0, kSun);
  EXPECT_NEAR(s.load_demand_w, 25500.0, 1.0e-6);
  EXPECT_EQ(find_load(s, "lab_payloads_a"), nullptr);
}

TEST(EpsDistribution, DdcuOverloadShedsLowestPriorityFirst)
{
  EpsParameters p = zero_beta();
  p.baseline_loads = {
    {"critical", 3000.0, 0, 0},
    {"life_support", 2000.0, 1, 0},
    {"payload", 2000.0, 3, 0},
  };
  EpsSystem eps(p);
  const EpsState & s = eps.step(1.0, kSun);
  EXPECT_TRUE(s.ddcu_overload);
  EXPECT_TRUE(s.load_shed);
  EXPECT_FALSE(s.healthy);
  EXPECT_EQ(s.shed_loads, 1);
  EXPECT_TRUE(find_load(s, "payload")->shed);
  EXPECT_DOUBLE_EQ(find_load(s, "critical")->served_w, 3000.0);
  EXPECT_DOUBLE_EQ(find_load(s, "life_support")->served_w, 2000.0);
  EXPECT_EQ(s.ddcus[0].mode, ddcu::DdcuMode::OVERLOAD);
  EXPECT_NEAR(s.ddcus[0].output_power_w, 5000.0, 1.0e-6);
}

TEST(EpsDistribution, StationOverloadShedsAndRecovers)
{
  EpsSystem eps(zero_beta());
  eps.set_demand_scale(2.0);
  const EpsState & over = eps.step(1.0, kSun);
  EXPECT_TRUE(over.load_shed);
  EXPECT_TRUE(over.load_unserved);
  EXPECT_FALSE(over.healthy);
  // Every vehicle-critical load that fits a DDCU stays on.
  for (const auto & l : over.loads) {
    if (l.priority == 0 && l.requested_w <= eps.parameters().ddcu.rated_power_w) {
      EXPECT_FALSE(l.shed) << l.id;
    }
  }
  eps.set_demand_scale(1.0);
  EXPECT_TRUE(eps.step(1.0, kSun).healthy);
}

TEST(EpsDistribution, MbsuShortfallShedsOnlyItsOwnLoads)
{
  EpsParameters p = zero_beta();
  p.bcdu.max_discharge_current_a = 8.0;  // 1.2 kW per channel in eclipse
  EpsSystem eps(p);
  const EpsState & s = eps.step(1.0, kShadow);
  EXPECT_TRUE(s.load_shed);
  for (const auto & m : s.mbsus) {
    EXPECT_LE(m.supplied_w, m.capability_w + 1.0e-6);
  }
  // Shedding never dropped a more critical load while a less critical one on
  // the same MBSU kept running.
  for (const auto & a : s.loads) {
    for (const auto & b : s.loads) {
      const int ma = p.ddcu_mbsu[static_cast<std::size_t>(a.ddcu)];
      const int mb = p.ddcu_mbsu[static_cast<std::size_t>(b.ddcu)];
      if (ma == mb && a.shed && !b.shed) {
        EXPECT_GE(a.priority, b.priority) << a.id << " shed while " << b.id << " served";
      }
    }
  }
}

TEST(EpsDistribution, ChannelLossIsReroutedWithinTheMbsu)
{
  EpsSystem eps(zero_beta());
  eps.set_channel_offline(0, true);
  const EpsState & one = eps.step(1.0, kShadow);
  EXPECT_NEAR(one.load_served_w, 30000.0, 1.0e-6);
  EXPECT_DOUBLE_EQ(one.channels[0].load_served_w, 0.0);
  EXPECT_EQ(one.mbsus[0].available_sources, 2);
  EXPECT_FALSE(one.mbsus[0].redundancy_lost);
  EXPECT_TRUE(one.healthy);

  eps.set_channel_offline(1, true);
  const EpsState & two = eps.step(1.0, kShadow);
  EXPECT_NEAR(two.load_served_w, 30000.0, 1.0e-6);
  EXPECT_TRUE(two.redundancy_lost);
  EXPECT_FALSE(two.healthy);
  EXPECT_EQ(two.status_message, "MBSU source redundancy lost");

  eps.set_channel_offline(2, true);
  const EpsState & none = eps.step(1.0, kShadow);
  EXPECT_TRUE(none.mbsus[0].unstable);
  EXPECT_TRUE(none.mbsu_unstable);
  EXPECT_TRUE(none.load_shed);
  EXPECT_EQ(none.ddcus[0].mode, ddcu::DdcuMode::OFF);
  EXPECT_EQ(none.ddcus[1].mode, ddcu::DdcuMode::OFF);
  // The other three MBSUs keep their loads.
  for (const auto & l : none.loads) {
    const int m = eps.parameters().ddcu_mbsu[static_cast<std::size_t>(l.ddcu)];
    EXPECT_EQ(l.shed, m == 0) << l.id;
  }
  for (int c = 0; c < 3; ++c) {
    eps.set_channel_offline(c, false);
  }
  EXPECT_TRUE(eps.step(1.0, kShadow).healthy);
}

TEST(EpsDistribution, DdcuFaultDropsItsLoadsAndRejectsNewOnes)
{
  EpsSystem eps(zero_beta());
  eps.step(1.0, kSun);
  eps.set_ddcu_fault(0, true);
  const EpsState & s = eps.step(1.0, kSun);
  EXPECT_TRUE(s.ddcu_fault);
  EXPECT_EQ(s.ddcus[0].mode, ddcu::DdcuMode::TRIPPED);
  for (const auto & l : s.loads) {
    EXPECT_EQ(l.shed, l.ddcu == 0) << l.id;
  }
  EXPECT_FALSE(eps.request_load("on_faulted", 500.0, 1, 0).accepted);
  EXPECT_TRUE(eps.request_load("elsewhere", 500.0, 1).accepted);

  eps.set_ddcu_fault(0, false);
  eps.release_load("elsewhere");
  EXPECT_TRUE(eps.step(1.0, kSun).healthy);
}

TEST(EpsDistribution, MbsuFaultIsolatesItsDdcus)
{
  EpsSystem eps(zero_beta());
  eps.set_mbsu_fault(2, true);
  const EpsState & s = eps.step(1.0, kSun);
  EXPECT_TRUE(s.mbsus[2].faulted);
  EXPECT_TRUE(s.mbsu_unstable);
  EXPECT_EQ(s.ddcus[4].mode, ddcu::DdcuMode::OFF);
  EXPECT_EQ(s.ddcus[5].mode, ddcu::DdcuMode::OFF);
  EXPECT_FALSE(s.healthy);
}

TEST(EpsDistribution, DistributionEnergyIsConservedOverAnOrbit)
{
  EpsSystem eps(zero_beta());
  for (int i = 0; i < 5546; i += 5) {
    const bool eclipse = i > 3600;
    const EpsState & s = eps.step(5.0, EpsInput{eclipse ? 0.0 : 1361.0, eclipse});
    double ddcu_in = 0.0;
    for (const auto & d : s.ddcus) {
      ddcu_in += d.input_power_w;
    }
    EXPECT_NEAR(ddcu_in, s.primary_load_w, 1.0e-3);
    EXPECT_NEAR(s.primary_load_w, s.load_served_w + s.distribution_loss_w, 1.0e-3);
  }
}
