#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

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
      "  conductance: 100.0\n";
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

TEST(StationThermalNetwork, ShippedConfigSettlesNearCoolantLoop)
{
  auto net = ThermalNetwork::load_from_yaml(
    std::string(SSOS_THERMAL_SOURCE_DIR) + "/config/station_thermal_nodes.yaml");
  ASSERT_EQ(net.nodes().count("itcs_sink"), 1u);
  ASSERT_EQ(net.nodes().count("itcs_coldplate"), 1u);
  // ~31 kW of station load into the cabin, ~2.5 kW of EPS losses on the plate.
  net.set_external_power("base_link", 30000.0);
  net.set_external_power("itcs_coldplate", 2500.0);
  for (int i = 0; i < 2000; ++i) {
    net.step(0.5);
  }
  EXPECT_NEAR(net.node_temperature("base_link"), 17.0 + (30000.0 + 1380.0 + 20.0) / 6000.0,
              0.05);
  EXPECT_NEAR(net.node_temperature("itcs_coldplate"), 17.0 + 2500.0 / 1000.0, 0.05);
}
