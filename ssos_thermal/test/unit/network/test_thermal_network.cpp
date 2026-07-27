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
