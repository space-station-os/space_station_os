#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "ssos_eclss/nodes/ars_node.hpp"

using namespace ssos_eclss::nodes;
using namespace std::chrono_literals;

class ArsNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<ArsNode>();
  }

  void spin_for(std::chrono::milliseconds d)
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_->get_node_base_interface());
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end && rclcpp::ok()) {
      exec.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  std::shared_ptr<ArsNode> node_;
};

TEST_F(ArsNodeTest, ConfigureActivateDeactivateCleanup)
{
  using lifecycle_msgs::msg::State;
  // Fast step rate so the timer fires several times within the spin window.
  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 30.0));
  EXPECT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  // Let the step timer run several cycles.
  spin_for(500ms);
  EXPECT_GT(node_->last_co2_removal_kg_day(), 0.0);
  EXPECT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(ArsNodeTest, RejectsInvalidParameterWhileConfigured)
{
  node_->configure();
  // Toth exponent out of range must be rejected by the set-parameters callback.
  const auto results =
    node_->set_parameters({rclcpp::Parameter("ars.isotherm.co2_13x.t0", 2.0)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_FALSE(results[0].successful);
}

TEST_F(ArsNodeTest, AcceptsValidDynamicParameter)
{
  node_->configure();
  const auto results =
    node_->set_parameters({rclcpp::Parameter("ars.heater.total_power_w", 800.0)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_TRUE(results[0].successful);
  EXPECT_NEAR(node_->get_parameter("ars.heater.total_power_w").as_double(), 800.0, 1e-9);
}
