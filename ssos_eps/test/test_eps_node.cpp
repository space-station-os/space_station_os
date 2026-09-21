#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ssos_eps/eps_node.hpp"

using namespace std::chrono_literals;

class EpsNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<ssos_eps::EpsNode>();
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_->get_node_base_interface());

    const auto end =
      std::chrono::steady_clock::now() + duration;

    while (
      std::chrono::steady_clock::now() < end &&
      rclcpp::ok())
    {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  std::shared_ptr<ssos_eps::EpsNode> node_;
};

TEST_F(EpsNodeTest, ConfigureActivateDeactivateCleanup)
{
  using lifecycle_msgs::msg::State;

  node_->set_parameter(
    rclcpp::Parameter("step_rate_hz", 30.0));

  EXPECT_EQ(
    node_->configure().id(),
    State::PRIMARY_STATE_INACTIVE);

  EXPECT_EQ(
    node_->activate().id(),
    State::PRIMARY_STATE_ACTIVE);

  spin_for(300ms);

  const auto & state = node_->last_state();

  EXPECT_GT(state.load_demand_w, 0.0);
  EXPECT_GE(state.battery_soc, 0.0);
  EXPECT_LE(state.battery_soc, 1.0);

  EXPECT_EQ(
    node_->deactivate().id(),
    State::PRIMARY_STATE_INACTIVE);

  EXPECT_EQ(
    node_->cleanup().id(),
    State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(EpsNodeTest, FrozenSimulationTimeDoesNotAdvanceBatterySoc)
{
  using lifecycle_msgs::msg::State;

  node_->set_parameter(
    rclcpp::Parameter("use_sim_time", true));

  node_->set_parameter(
    rclcpp::Parameter("step_rate_hz", 30.0));

  node_->set_parameter(
    rclcpp::Parameter("initial_battery_soc", 0.50));

  EXPECT_EQ(
    node_->configure().id(),
    State::PRIMARY_STATE_INACTIVE);

  EXPECT_EQ(
    node_->activate().id(),
    State::PRIMARY_STATE_ACTIVE);

  // Allow the ROS time source to settle after enabling simulated time.
  // No /clock messages are published, so ROS time then remains frozen.
  spin_for(300ms);

  const double soc_before = node_->last_state().battery_soc;

  // Wall timers continue firing, but frozen ROS time must not advance
  // the battery energy state.
  spin_for(300ms);

  const double soc_after = node_->last_state().battery_soc;

  EXPECT_DOUBLE_EQ(
    soc_after,
    soc_before);

  EXPECT_EQ(
    node_->deactivate().id(),
    State::PRIMARY_STATE_INACTIVE);

  EXPECT_EQ(
    node_->cleanup().id(),
    State::PRIMARY_STATE_UNCONFIGURED);
}
