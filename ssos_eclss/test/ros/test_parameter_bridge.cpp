#include <gtest/gtest.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "ssos_eclss/nodes/eclss_parameter_bridge.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::nodes;

class ParameterBridgeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("bridge_test_node");
    bridge_ = std::make_unique<EclssParameterBridge>(node_.get());
    bridge_->declare_ars_parameters(ars::default_ars_parameters());
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::unique_ptr<EclssParameterBridge> bridge_;
};

TEST_F(ParameterBridgeTest, DeclaresAndReadsDefaults)
{
  const ars::ArsParameters p = bridge_->read_ars_parameters();
  const ars::ArsParameters d = ars::default_ars_parameters();
  EXPECT_NEAR(p.heater.total_power_w, d.heater.total_power_w, 1.0e-9);
  EXPECT_NEAR(p.operating.inlet_flow_scfm, d.operating.inlet_flow_scfm, 1.0e-9);
  EXPECT_EQ(p.adsorbent_bed.n_cells, d.adsorbent_bed.n_cells);
}

TEST_F(ParameterBridgeTest, ReadsOverriddenValue)
{
  node_->set_parameter(rclcpp::Parameter("ars.heater.total_power_w", 850.0));
  const ars::ArsParameters p = bridge_->read_ars_parameters();
  EXPECT_NEAR(p.heater.total_power_w, 850.0, 1.0e-9);
}

TEST_F(ParameterBridgeTest, ValidatesTothExponentRange)
{
  // Valid t0 accepted.
  auto ok = bridge_->validate({rclcpp::Parameter("ars.isotherm.co2_13x.t0", 0.5)});
  EXPECT_TRUE(ok.successful);
  // t0 > 1 rejected.
  auto bad = bridge_->validate({rclcpp::Parameter("ars.isotherm.co2_13x.t0", 1.5)});
  EXPECT_FALSE(bad.successful);
  EXPECT_FALSE(bad.reason.empty());
  // t0 <= 0 rejected.
  auto bad2 = bridge_->validate({rclcpp::Parameter("ars.isotherm.co2_13x.t0", 0.0)});
  EXPECT_FALSE(bad2.successful);
}

TEST_F(ParameterBridgeTest, ValidatesVoidageRange)
{
  EXPECT_FALSE(
    bridge_->validate({rclcpp::Parameter("ars.bed.adsorbent.voidage", 1.5)}).successful);
  EXPECT_TRUE(
    bridge_->validate({rclcpp::Parameter("ars.bed.adsorbent.voidage", 0.4)}).successful);
}

TEST_F(ParameterBridgeTest, ValidatesNonNegativeFlow)
{
  EXPECT_FALSE(
    bridge_->validate({rclcpp::Parameter("ars.operating.inlet_flow_scfm", -1.0)})
      .successful);
}

TEST_F(ParameterBridgeTest, ValidatesTemperaturePositive)
{
  EXPECT_FALSE(
    bridge_->validate({rclcpp::Parameter("ars.operating.cabin_temp_k", -5.0)})
      .successful);
}

TEST_F(ParameterBridgeTest, EfficiencyBoundedToUnitInterval)
{
  EXPECT_FALSE(
    bridge_->validate({rclcpp::Parameter("ars.efficiency.capture_efficiency", 1.4)})
      .successful);
  EXPECT_TRUE(
    bridge_->validate({rclcpp::Parameter("ars.efficiency.capture_efficiency", 0.84)})
      .successful);
}

TEST_F(ParameterBridgeTest, StaticParameterClassification)
{
  EXPECT_TRUE(EclssParameterBridge::is_static_parameter("ars.bed.adsorbent.length"));
  EXPECT_FALSE(EclssParameterBridge::is_static_parameter("ars.heater.total_power_w"));
}
