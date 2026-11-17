#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "ssos_eps/nodes/eps_parameter_bridge.hpp"

using ssos_eps::nodes::EpsParameterBridge;
using ssos_eps::system::default_eps_parameters;

class EpsParameterBridgeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("eps_bridge_test");
    bridge_ = std::make_unique<EpsParameterBridge>(node_.get());
    bridge_->declare_parameters(default_eps_parameters());
  }

  bool accepts(const std::vector<rclcpp::Parameter> & params)
  {
    return bridge_->validate(params).successful;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::unique_ptr<EpsParameterBridge> bridge_;
};

TEST_F(EpsParameterBridgeTest, ReadsBackDefaults)
{
  const auto d = default_eps_parameters();
  const auto p = bridge_->read_parameters();
  EXPECT_EQ(p.n_channels, d.n_channels);
  EXPECT_EQ(p.orus_per_channel, d.orus_per_channel);
  EXPECT_DOUBLE_EQ(p.solar.array.area_m2, d.solar.array.area_m2);
  EXPECT_DOUBLE_EQ(p.solar.ssu.voltage_setpoint_v, d.solar.ssu.voltage_setpoint_v);
  EXPECT_DOUBLE_EQ(p.battery.capacity_ah, d.battery.capacity_ah);
  EXPECT_EQ(p.battery.cell_ocv_v, d.battery.cell_ocv_v);
  EXPECT_DOUBLE_EQ(p.bcdu.charge_enter_v, d.bcdu.charge_enter_v);
  EXPECT_EQ(p.solar.array.use_fixed_beta, d.solar.array.use_fixed_beta);
}

TEST_F(EpsParameterBridgeTest, ReadsOverriddenValues)
{
  node_->set_parameter(rclcpp::Parameter("solar.array.cell_efficiency", 0.2));
  node_->set_parameter(rclcpp::Parameter("eps.n_channels", 8));
  const auto p = bridge_->read_parameters();
  EXPECT_DOUBLE_EQ(p.solar.array.cell_efficiency, 0.2);
  EXPECT_EQ(p.n_channels, 8);
}

TEST_F(EpsParameterBridgeTest, RejectsOutOfRangeValues)
{
  EXPECT_FALSE(accepts({rclcpp::Parameter("solar.array.cell_efficiency", 0.0)}));
  EXPECT_FALSE(accepts({rclcpp::Parameter("solar.array.degradation_factor", 1.2)}));
  EXPECT_FALSE(accepts({rclcpp::Parameter("battery.capacity_ah", -5.0)}));
  EXPECT_FALSE(accepts({rclcpp::Parameter("bcdu.discharge_efficiency", 0.0)}));
  EXPECT_FALSE(accepts({rclcpp::Parameter("eps.n_channels", 0)}));
  EXPECT_TRUE(accepts({rclcpp::Parameter("solar.array.degradation_factor", 0.0)}));
  EXPECT_TRUE(accepts({rclcpp::Parameter("battery.capacity_ah", 100.0)}));
}

TEST_F(EpsParameterBridgeTest, RejectsInvertedHysteresis)
{
  EXPECT_FALSE(accepts({rclcpp::Parameter("bcdu.discharge_enter_v", 165.0)}));
  EXPECT_TRUE(accepts({rclcpp::Parameter("bcdu.discharge_enter_v", 165.0),
      rclcpp::Parameter("bcdu.charge_enter_v", 170.0)}));
}

TEST_F(EpsParameterBridgeTest, RejectsRegulationOutsideLimits)
{
  EXPECT_FALSE(accepts({rclcpp::Parameter("bcdu.regulation_voltage_v", 190.0)}));
  EXPECT_TRUE(accepts({rclcpp::Parameter("bcdu.regulation_voltage_v", 140.0)}));
}

TEST_F(EpsParameterBridgeTest, RejectsInvertedCellVoltageLimits)
{
  EXPECT_FALSE(accepts({rclcpp::Parameter("battery.min_cell_voltage_v", 4.2)}));
}

TEST_F(EpsParameterBridgeTest, ValidatesOcvTable)
{
  EXPECT_FALSE(accepts({rclcpp::Parameter("battery.soc_points",
      std::vector<double>{0.0, 1.0})}));
  EXPECT_FALSE(accepts({rclcpp::Parameter("battery.soc_points",
      std::vector<double>{0.0, 0.6, 0.5}),
    rclcpp::Parameter("battery.cell_ocv_v", std::vector<double>{3.0, 3.5, 4.0})}));
  EXPECT_TRUE(accepts({rclcpp::Parameter("battery.soc_points",
      std::vector<double>{0.0, 0.5, 1.0}),
    rclcpp::Parameter("battery.cell_ocv_v", std::vector<double>{3.0, 3.7, 4.1})}));
}

TEST_F(EpsParameterBridgeTest, ClassifiesParameters)
{
  EXPECT_TRUE(EpsParameterBridge::is_static_parameter("eps.n_channels"));
  EXPECT_TRUE(EpsParameterBridge::is_static_parameter("battery.capacity_ah"));
  EXPECT_FALSE(EpsParameterBridge::is_static_parameter("bcdu.charge_enter_v"));
  EXPECT_TRUE(EpsParameterBridge::is_physics_parameter("solar.ssu.efficiency"));
  EXPECT_FALSE(EpsParameterBridge::is_physics_parameter("load_demand_w"));
}

TEST_F(EpsParameterBridgeTest, ReadsTopologyAndBaselineLoads)
{
  const auto p = bridge_->read_parameters();
  const auto d = default_eps_parameters();
  EXPECT_EQ(p.n_mbsu, d.n_mbsu);
  EXPECT_EQ(p.channel_mbsu, d.channel_mbsu);
  EXPECT_EQ(p.ddcu_mbsu, d.ddcu_mbsu);
  ASSERT_EQ(p.baseline_loads.size(), d.baseline_loads.size());
  EXPECT_EQ(p.baseline_loads.front().id, d.baseline_loads.front().id);
  EXPECT_DOUBLE_EQ(p.baseline_loads.back().power_w, d.baseline_loads.back().power_w);
  EXPECT_DOUBLE_EQ(p.ddcu.rated_power_w, d.ddcu.rated_power_w);
  EXPECT_DOUBLE_EQ(p.demand_scale, 1.0);
}

TEST_F(EpsParameterBridgeTest, ValidatesDistributionParameters)
{
  EXPECT_FALSE(accepts({rclcpp::Parameter("ddcu.efficiency", 1.2)}));
  EXPECT_FALSE(accepts({rclcpp::Parameter("ddcu.input_min_v", 180.0)}));
  EXPECT_FALSE(accepts({rclcpp::Parameter("ddcu.reset_temp_c", 75.0)}));
  EXPECT_FALSE(accepts({rclcpp::Parameter("mbsu.min_source_voltage_v", 200.0)}));
  EXPECT_FALSE(accepts({rclcpp::Parameter("loads.demand_scale", -0.5)}));
  EXPECT_TRUE(accepts({rclcpp::Parameter("loads.demand_scale", 1.5)}));
  EXPECT_FALSE(accepts({rclcpp::Parameter("eps.channel_mbsu", std::vector<int64_t>{0, 1})}));
  EXPECT_FALSE(accepts({rclcpp::Parameter(
      "eps.ddcu_mbsu", std::vector<int64_t>{0, 0, 1, 1, 2, 2, 3, 9})}));
  EXPECT_FALSE(accepts({rclcpp::Parameter(
      "loads.ids", std::vector<std::string>{"a", "b"})}));
  EXPECT_TRUE(EpsParameterBridge::is_static_parameter("loads.ids"));
  EXPECT_TRUE(EpsParameterBridge::is_static_parameter("eps.ddcu_mbsu"));
  EXPECT_FALSE(EpsParameterBridge::is_static_parameter("ddcu.rated_power_w"));
  EXPECT_TRUE(EpsParameterBridge::is_physics_parameter("loads.demand_scale"));
}
