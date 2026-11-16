#include <gtest/gtest.h>

#include <vector>

#include "ssos_eps/mbsu/mbsu_model.hpp"

using namespace ssos_eps::mbsu;

namespace
{
std::vector<MbsuSource> three_sources()
{
  return {
    MbsuSource{0, true, 160.0, 6000.0},
    MbsuSource{1, true, 160.0, 6000.0},
    MbsuSource{2, true, 160.0, 3000.0},
  };
}
}  // namespace

TEST(Mbsu, SplitsDemandByCapability)
{
  MbsuModel mbsu;
  const MbsuState & s = mbsu.route(three_sources(), 7500.0);
  EXPECT_FALSE(s.unstable);
  EXPECT_FALSE(s.redundancy_lost);
  EXPECT_EQ(s.available_sources, 3);
  EXPECT_DOUBLE_EQ(s.capability_w, 15000.0);
  EXPECT_DOUBLE_EQ(s.supplied_w, 7500.0);
  EXPECT_DOUBLE_EQ(s.shortfall_w, 0.0);
  EXPECT_NEAR(s.allocation_w[0], 3000.0, 1.0e-9);
  EXPECT_NEAR(s.allocation_w[1], 3000.0, 1.0e-9);
  EXPECT_NEAR(s.allocation_w[2], 1500.0, 1.0e-9);
  EXPECT_NEAR(s.bus_voltage_v, 160.0, 1.0e-9);
  EXPECT_EQ(s.status_message, "nominal");
}

TEST(Mbsu, LostChannelIsReroutedToTheOthers)
{
  MbsuModel mbsu;
  auto sources = three_sources();
  sources[0].connected = false;
  const MbsuState & s = mbsu.route(sources, 7500.0);
  EXPECT_EQ(s.available_sources, 2);
  EXPECT_FALSE(s.source_available[0]);
  EXPECT_DOUBLE_EQ(s.allocation_w[0], 0.0);
  EXPECT_NEAR(s.allocation_w[1] + s.allocation_w[2], 7500.0, 1.0e-9);
  EXPECT_FALSE(s.redundancy_lost);
}

TEST(Mbsu, LowVoltageSourceIsNotUsed)
{
  MbsuModel mbsu;
  auto sources = three_sources();
  sources[1].voltage_v = 110.0;
  const MbsuState & s = mbsu.route(sources, 1000.0);
  EXPECT_FALSE(s.source_available[1]);
  EXPECT_EQ(s.available_sources, 2);
}

TEST(Mbsu, SingleSourceLosesRedundancy)
{
  MbsuModel mbsu;
  auto sources = three_sources();
  sources[0].connected = false;
  sources[1].capability_w = 0.0;
  const MbsuState & s = mbsu.route(sources, 1000.0);
  EXPECT_EQ(s.available_sources, 1);
  EXPECT_FALSE(s.unstable);
  EXPECT_TRUE(s.redundancy_lost);
  EXPECT_EQ(s.status_message, "source redundancy lost");
}

TEST(Mbsu, NoSourceMakesBusUnstable)
{
  MbsuModel mbsu;
  auto sources = three_sources();
  for (auto & src : sources) {
    src.capability_w = 0.0;
  }
  const MbsuState & s = mbsu.route(sources, 1000.0);
  EXPECT_TRUE(s.unstable);
  EXPECT_DOUBLE_EQ(s.supplied_w, 0.0);
  EXPECT_DOUBLE_EQ(s.shortfall_w, 1000.0);
  EXPECT_DOUBLE_EQ(s.bus_voltage_v, 0.0);
}

TEST(Mbsu, DemandAboveCapabilityReportsShortfall)
{
  MbsuModel mbsu;
  const MbsuState & s = mbsu.route(three_sources(), 20000.0);
  EXPECT_DOUBLE_EQ(s.supplied_w, 15000.0);
  EXPECT_DOUBLE_EQ(s.shortfall_w, 5000.0);
  EXPECT_EQ(s.status_message, "demand exceeds source capability");
}

TEST(Mbsu, FaultIsolatesBus)
{
  MbsuModel mbsu;
  mbsu.set_fault(true);
  const MbsuState & s = mbsu.route(three_sources(), 1000.0);
  EXPECT_TRUE(s.faulted);
  EXPECT_TRUE(s.unstable);
  EXPECT_DOUBLE_EQ(s.supplied_w, 0.0);
  mbsu.set_fault(false);
  EXPECT_FALSE(mbsu.route(three_sources(), 1000.0).unstable);
}
