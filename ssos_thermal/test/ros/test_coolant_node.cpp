#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

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

namespace
{

using CoolantStatus = space_station_interfaces::msg::CoolantStatus;
using CoolantAction = space_station_interfaces::action::Coolant;

class CoolantStatusTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<CoolantNode>();
    observer_ = std::make_shared<rclcpp::Node>("coolant_status_observer");
    sub_ = observer_->create_subscription<CoolantStatus>(
      "/thermal/coolant/status", 10,
      [this](const CoolantStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        received_.push_back(*msg);
      });
    exec_.add_node(node_->get_node_base_interface());
    exec_.add_node(observer_);
  }

  void spin_until(const std::function<bool()> & done, std::chrono::milliseconds timeout)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end && rclcpp::ok() && !done()) {
      exec_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  std::vector<CoolantStatus> received()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_;
  }

  std::shared_ptr<CoolantNode> node_;
  rclcpp::Node::SharedPtr observer_;
  rclcpp::Subscription<CoolantStatus>::SharedPtr sub_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::mutex mutex_;
  std::vector<CoolantStatus> received_;
};

}  // namespace

TEST_F(CoolantStatusTest, PublishesIdleStatusWithoutAnyGoal)
{
  node_->configure();
  node_->activate();
  spin_until([this]() {return !received().empty();}, 3s);

  const auto msgs = received();
  ASSERT_FALSE(msgs.empty());
  EXPECT_FALSE(msgs.back().active);
  EXPECT_DOUBLE_EQ(msgs.back().internal_temp_c, 25.0);  // target_temp_c default
  EXPECT_DOUBLE_EQ(msgs.back().vented_heat_kj, 0.0);
}

TEST_F(CoolantStatusTest, ReportsActiveDuringGoalThenIdle)
{
  node_->configure();
  node_->activate();

  auto client = rclcpp_action::create_client<CoolantAction>(observer_, "coolant_heat_transfer");
  ASSERT_TRUE(client->wait_for_action_server(2s));

  CoolantAction::Goal goal;
  goal.input_temperature_c = 30.0;
  goal.component_id = "test_component";
  client->async_send_goal(goal);

  auto saw_active_then_idle = [this]() {
      bool active_seen = false;
      for (const auto & m : received()) {
        if (m.active && m.component_id == "test_component") {
          active_seen = true;
        } else if (active_seen && !m.active) {
          return true;
        }
      }
      return false;
    };
  spin_until(saw_active_then_idle, 5s);
  EXPECT_TRUE(saw_active_then_idle());

  const auto msgs = received();
  ASSERT_FALSE(msgs.empty());
  EXPECT_LE(msgs.back().internal_temp_c, 25.5);
}

TEST_F(CoolantStatusTest, DeactivateDuringGoalAbortsPromptly)
{
  using lifecycle_msgs::msg::State;
  node_->configure();
  node_->activate();

  auto client = rclcpp_action::create_client<CoolantAction>(observer_, "coolant_heat_transfer");
  ASSERT_TRUE(client->wait_for_action_server(2s));

  // 100 degC -> 25 degC at 2.5 degC per 100 ms step takes ~3 s if left alone.
  CoolantAction::Goal goal;
  goal.input_temperature_c = 100.0;
  goal.component_id = "long_cycle";
  client->async_send_goal(goal);

  spin_until([this]() {
      for (const auto & m : received()) {
        if (m.active) {
          return true;
        }
      }
      return false;
    }, 2s);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
  EXPECT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}
