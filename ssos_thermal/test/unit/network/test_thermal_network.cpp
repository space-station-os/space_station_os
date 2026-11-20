#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <unordered_map>

#include "ssos_thermal/network/thermal_network.hpp"

using ssos_thermal::network::ThermalNetwork;

namespace
{

class ThermalNetworkTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    path_ = (std::filesystem::temp_directory_path() /
      "ssos_thermal_test_network.yaml").string();
    std::ofstream out(path_);
    out <<
      "- node_name: \"A\"\n"
      "  parent_link: \"base_link\"\n"
      "  heat_capacity: 100.0\n"
      "  internal_power: 50.0\n"
      "  conductance: 1.0\n"
      "\n"
      "- node_name: \"B\"\n"
      "  parent_link: \"A\"\n"
      "  heat_capacity: 100.0\n"
      "  internal_power: 0.0\n"
      "  conductance: 1.0\n";
  }

  void TearDown() override
  {
    std::filesystem::remove(path_);
  }

  std::string path_;
};

}  // namespace

TEST_F(ThermalNetworkTest, LoadsExpectedNodeAndLinkCount)
{
  ThermalNetwork net = ThermalNetwork::load_from_yaml(path_);
  EXPECT_EQ(net.nodes().size(), 2u);
  EXPECT_EQ(net.links().size(), 2u);
}

TEST_F(ThermalNetworkTest, NodesWithoutInitialTemperatureStartAtTheReference)
{
  ThermalNetwork net = ThermalNetwork::load_from_yaml(path_, 15.0);
  EXPECT_DOUBLE_EQ(net.node_temperature("A"), 15.0);
  EXPECT_DOUBLE_EQ(net.node_temperature("B"), 15.0);
  EXPECT_FALSE(net.nodes().at("A").max_temperature.has_value());
}

TEST_F(ThermalNetworkTest, StepWarmsNodeWithNoOtherHeatSource)
{
  ThermalNetwork net = ThermalNetwork::load_from_yaml(path_);
  net.set_all_temperatures(20.0);
  const double a_before = net.node_temperature("A");
  net.step(1.0);
  const double a_after = net.node_temperature("A");
  // A has 50W internal power and B (its only link partner) starts at the
  // same temperature, so A must warm up.
  EXPECT_GT(a_after, a_before);
}

TEST_F(ThermalNetworkTest, HottestPicksTheNodeWithInternalPower)
{
  ThermalNetwork net = ThermalNetwork::load_from_yaml(path_);
  net.set_all_temperatures(20.0);
  for (int i = 0; i < 50; ++i) {
    net.step(1.0);
  }
  const auto hottest = net.hottest();
  EXPECT_EQ(hottest.name, "A");
}

namespace
{

class ThermalNetworkRootTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    path_ = (std::filesystem::temp_directory_path() /
      "ssos_thermal_test_root_network.yaml").string();
    std::ofstream out(path_);
    out <<
      "- node_name: \"base_link\"\n"
      "  parent_link: \"\"\n"
      "  heat_capacity: 1000.0\n"
      "  internal_power: 0.0\n"
      "  conductance: 0.0\n"
      "\n"
      "- node_name: \"Panel\"\n"
      "  parent_link: \"base_link\"\n"
      "  heat_capacity: 100.0\n"
      "  internal_power: 50.0\n"
      "  conductance: 1.0\n";
  }

  void TearDown() override
  {
    std::filesystem::remove(path_);
  }

  std::string path_;
};

}  // namespace

TEST_F(ThermalNetworkRootTest, EmptyParentLinkCreatesNoLinkForTheRoot)
{
  // 2 nodes, but only 1 link (Panel -> base_link); base_link's own
  // parent_link is empty, so it gets no link of its own.
  ThermalNetwork net = ThermalNetwork::load_from_yaml(path_);
  EXPECT_EQ(net.nodes().size(), 2u);
  EXPECT_EQ(net.links().size(), 1u);
}

TEST_F(ThermalNetworkRootTest, RootNodeActuallyReceivesConductedHeat)
{
  // Regression test for the pre-fix behavior where a node's parent_link
  // was never itself a declared node_name, so conduction to it was inert.
  // Here base_link IS declared, so heat from Panel's internal_power must
  // actually raise base_link's temperature over time.
  ThermalNetwork net = ThermalNetwork::load_from_yaml(path_);
  net.set_all_temperatures(20.0);
  const double base_before = net.node_temperature("base_link");
  for (int i = 0; i < 50; ++i) {
    net.step(1.0);
  }
  const double base_after = net.node_temperature("base_link");
  EXPECT_GT(base_after, base_before);
}

TEST_F(ThermalNetworkRootTest, LinkHeatFlowIsZeroAtEqualTemperatures)
{
  ThermalNetwork net = ThermalNetwork::load_from_yaml(path_);
  net.set_all_temperatures(40.0);
  ASSERT_EQ(net.links().size(), 1u);
  EXPECT_DOUBLE_EQ(net.link_heat_flow(net.links().front()), 0.0);
}

TEST_F(ThermalNetworkRootTest, LinkHeatFlowUsesBothConnectedNodeTemperatures)
{
  // Regression test: telemetry used to compare each node against a fixed
  // 20 degC reference instead of the node at the other end of the link.
  // Panel (50 W) heats faster than base_link, so heat flows Panel -> base_link.
  ThermalNetwork net = ThermalNetwork::load_from_yaml(path_);
  net.set_all_temperatures(20.0);
  for (int i = 0; i < 20; ++i) {
    net.step(1.0);
  }
  const auto & link = net.links().front();
  ASSERT_EQ(link.from, "Panel");
  ASSERT_EQ(link.to, "base_link");

  const double t_panel = net.node_temperature("Panel");
  const double t_base = net.node_temperature("base_link");
  ASSERT_GT(t_panel, t_base);
  EXPECT_DOUBLE_EQ(net.link_heat_flow(link), link.conductance * (t_panel - t_base));
  EXPECT_GT(net.link_heat_flow(link), 0.0);
}

TEST_F(ThermalNetworkTest, LinkToUndeclaredNodeCarriesNoHeat)
{
  // A's parent_link "base_link" is not a declared node in this fixture, so
  // compute_dTdt() exchanges no heat over that link; telemetry must agree.
  ThermalNetwork net = ThermalNetwork::load_from_yaml(path_);
  net.set_all_temperatures(60.0);
  for (const auto & link : net.links()) {
    if (link.to == "base_link") {
      EXPECT_DOUBLE_EQ(net.link_heat_flow(link), 0.0);
    }
  }
}

namespace
{

class ThermalSinkTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    path_ = (std::filesystem::temp_directory_path() /
      "ssos_thermal_test_sink_network.yaml").string();
    std::ofstream out(path_);
    out <<
      "- node_name: \"sink\"\n"
      "  parent_link: \"\"\n"
      "  heat_capacity: 1.0e6\n"
      "  internal_power: 0.0\n"
      "  conductance: 0.0\n"
      "  fixed_temperature_c: 17.0\n"
      "\n"
      "- node_name: \"plate\"\n"
      "  parent_link: \"sink\"\n"
      "  heat_capacity: 500.0\n"
      "  internal_power: 0.0\n"
      "  conductance: 100.0\n"
      "  initial_temperature_c: 25.0\n"
      "  max_temperature_c: 30.0\n";
  }

  void TearDown() override
  {
    std::filesystem::remove(path_);
  }

  std::string path_;
};

}  // namespace

TEST_F(ThermalSinkTest, FixedNodeHoldsItsTemperature)
{
  auto net = ThermalNetwork::load_from_yaml(path_);
  EXPECT_TRUE(net.nodes().at("sink").fixed);
  EXPECT_DOUBLE_EQ(net.node_temperature("sink"), 17.0);
  ASSERT_TRUE(net.set_external_power("plate", 2000.0));
  for (int i = 0; i < 200; ++i) {
    net.step(0.5);
  }
  EXPECT_DOUBLE_EQ(net.node_temperature("sink"), 17.0);
}

TEST_F(ThermalSinkTest, ExternalPowerSettlesAtSinkPlusPowerOverConductance)
{
  auto net = ThermalNetwork::load_from_yaml(path_);
  ASSERT_TRUE(net.set_external_power("plate", 1500.0));
  EXPECT_DOUBLE_EQ(net.total_power("plate"), 1500.0);
  // Time constant C/G = 5 s; 100 s is 20 time constants.
  for (int i = 0; i < 200; ++i) {
    net.step(0.5);
  }
  EXPECT_NEAR(net.node_temperature("plate"), 17.0 + 1500.0 / 100.0, 1.0e-3);

  net.clear_external_power();
  for (int i = 0; i < 200; ++i) {
    net.step(0.5);
  }
  EXPECT_NEAR(net.node_temperature("plate"), 17.0, 1.0e-3);
  EXPECT_FALSE(net.set_external_power("missing", 10.0));
}

TEST_F(ThermalSinkTest, BoundaryNodesAreLeftOutOfAggregates)
{
  auto net = ThermalNetwork::load_from_yaml(path_);
  net.set_all_temperatures(40.0);
  EXPECT_DOUBLE_EQ(net.node_temperature("sink"), 17.0);
  EXPECT_DOUBLE_EQ(net.node_temperature("plate"), 40.0);
  EXPECT_DOUBLE_EQ(net.average_temperature(), 40.0);
  EXPECT_EQ(net.hottest().name, "plate");
}

TEST_F(ThermalSinkTest, InitialTemperatureAndLimitComeFromYaml)
{
  auto net = ThermalNetwork::load_from_yaml(path_);
  EXPECT_DOUBLE_EQ(net.node_temperature("plate"), 25.0);
  EXPECT_DOUBLE_EQ(net.nodes().at("plate").max_temperature.value(), 30.0);
  EXPECT_DOUBLE_EQ(ThermalNetwork::load_from_yaml(path_).node_temperature("plate"), 25.0);
}

TEST_F(ThermalSinkTest, OverLimitUsesNodeLimitAndMargin)
{
  auto net = ThermalNetwork::load_from_yaml(path_);
  // plate's own 30 degC limit wins over the fallback; the sink is never checked.
  EXPECT_EQ(net.over_limit(20.0), "");
  net.set_all_temperatures(30.5);
  EXPECT_EQ(net.over_limit(500.0), "plate");
  // Once over, the caller passes the hysteresis margin: 29.5 still counts.
  net.set_all_temperatures(29.5);
  EXPECT_EQ(net.over_limit(500.0), "");
  EXPECT_EQ(net.over_limit(500.0, 1.0), "plate");
  net.set_all_temperatures(28.9);
  EXPECT_EQ(net.over_limit(500.0, 1.0), "");
}

TEST_F(ThermalSinkTest, DrillOverridesAreRestored)
{
  auto net = ThermalNetwork::load_from_yaml(path_);
  EXPECT_FALSE(net.set_fixed_temperature("plate", 30.0));
  EXPECT_TRUE(net.set_fixed_temperature("sink", 30.0));
  EXPECT_EQ(net.scale_conductance("plate", 0.1), 1u);
  EXPECT_EQ(net.scale_conductance("missing", 0.1), 0u);
  EXPECT_DOUBLE_EQ(net.node_temperature("sink"), 30.0);
  EXPECT_DOUBLE_EQ(net.links().front().conductance, 10.0);
  // Scaling is relative to the configured value, not compounded.
  net.scale_conductance("sink", 0.5);
  EXPECT_DOUBLE_EQ(net.links().front().conductance, 50.0);

  net.clear_overrides();
  EXPECT_DOUBLE_EQ(net.node_temperature("sink"), 17.0);
  EXPECT_DOUBLE_EQ(net.links().front().conductance, 100.0);
}

TEST(StationThermalNetwork, StartsSettledAndStaysWithinLimitsForADay)
{
  auto net = ThermalNetwork::load_from_yaml(
    std::string(SSOS_THERMAL_SOURCE_DIR) + "/config/station_thermal_nodes.yaml");
  ASSERT_EQ(net.nodes().count("itcs_sink"), 1u);
  ASSERT_EQ(net.nodes().count("itcs_coldplate"), 1u);
  std::unordered_map<std::string, double> initial;
  for (const auto & [name, node] : net.nodes()) {
    initial[name] = node.temperature;
  }
  // Nominal EPS loads: delivered power into the cabin, conversion losses on the plate.
  net.set_external_power("base_link", 26700.0);
  net.set_external_power("itcs_coldplate", 4700.0);
  for (int i = 0; i < 24 * 3600 * 2; ++i) {
    net.step(0.5);
    ASSERT_EQ(net.over_limit(500.0), "") << "at step " << i;
  }
  for (const auto & [name, t0] : initial) {
    EXPECT_NEAR(net.node_temperature(name), t0, 0.5) << name;
  }
}

// Every drill has to show up as a limit trip; times match docs/fault_catalog.md.
TEST(StationThermalNetwork, EveryDrillTripsALimit)
{
  const auto seconds_to_trip = [](const std::function<void(ThermalNetwork &)> & drill) {
      auto net = ThermalNetwork::load_from_yaml(
        std::string(SSOS_THERMAL_SOURCE_DIR) + "/config/station_thermal_nodes.yaml");
      net.set_external_power("base_link", 26700.0);
      net.set_external_power("itcs_coldplate", 4700.0);
      drill(net);
      for (int i = 1; i <= 2 * 3600 * 2; ++i) {
        net.step(0.5);
        if (net.over_limit(500.0) == "itcs_coldplate") {
          return 0.5 * i;
        }
      }
      return -1.0;
    };
  // itcs_loop_degraded: plate heads for 30.7 degC, time constant 50 s.
  EXPECT_NEAR(seconds_to_trip([](ThermalNetwork & net) {
      net.set_fixed_temperature("itcs_sink", 26.0);
    }), 60.0, 3.0);
  // coldplate_conductance_loss: plate heads for 30.4 degC, time constant 143 s.
  EXPECT_NEAR(seconds_to_trip([](ThermalNetwork & net) {
      net.scale_conductance("itcs_coldplate", 0.35);
    }), 183.0, 5.0);
}
