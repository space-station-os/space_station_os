#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ssos_thermal/nodes/coolant_node.hpp"

using namespace ssos_thermal::nodes;
using namespace std::chrono_literals;

namespace
{

void spin_node_for(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & base,
  std::chrono::milliseconds d)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(base);
  const auto end = std::chrono::steady_clock::now() + d;
  while (std::chrono::steady_clock::now() < end && rclcpp::ok()) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }
}

}  // namespace

class CoolantNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<CoolantNode>();
  }

  std::shared_ptr<CoolantNode> node_;
};

TEST_F(CoolantNodeTest, ConfigureActivateDeactivateCleanup)
{
  using lifecycle_msgs::msg::State;
  EXPECT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  spin_node_for(node_->get_node_base_interface(), 200ms);
  EXPECT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(CoolantNodeAutostartTest, ConfiguresAndActivatesWithoutExternalCall)
{
  using lifecycle_msgs::msg::State;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("autostart", true), rclcpp::Parameter("autostart_delay_ms", 50)});
  auto node = std::make_shared<CoolantNode>(options);

  spin_node_for(node->get_node_base_interface(), 500ms);
  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
}
