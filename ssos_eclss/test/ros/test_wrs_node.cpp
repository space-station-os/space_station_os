#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "ssos_eclss/nodes/wrs_node.hpp"

using namespace ssos_eclss::nodes;
using namespace std::chrono_literals;
using std_msgs::msg::Float64;

class WrsNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<WrsNode>();
    feeder_ = std::make_shared<rclcpp::Node>("wrs_test_feeder");
    urine_ = feeder_->create_publisher<Float64>("/ssos/crew/urine_kg_day", 10);
    latent_ = feeder_->create_publisher<Float64>("/ssos/crew/latent_water_kg_day", 10);
  }

  void spin_and_feed(std::chrono::milliseconds d, double urine_kg_day)
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_->get_node_base_interface());
    exec.add_node(feeder_);
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end && rclcpp::ok()) {
      Float64 m;
      m.data = urine_kg_day; urine_->publish(m);
      m.data = 8.8; latent_->publish(m);
      exec.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  std::shared_ptr<WrsNode> node_;
  rclcpp::Node::SharedPtr feeder_;
  rclcpp::Publisher<Float64>::SharedPtr urine_;
  rclcpp::Publisher<Float64>::SharedPtr latent_;
};

TEST_F(WrsNodeTest, LifecycleAndInventories)
{
  using lifecycle_msgs::msg::State;
  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 30.0));
  EXPECT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  spin_and_feed(400ms, 6.0);  // nominal 4-crew urine load
  // Potable tank starts at the configured initial inventory (300 kg).
  EXPECT_GT(node_->potable_available_kg(), 290.0);
  EXPECT_LT(node_->potable_available_kg(), 320.0);
  // Wastewater accumulated some urine; conductivity stays finite.
  EXPECT_GE(node_->wastewater_kg(), 0.0);
  EXPECT_TRUE(std::isfinite(node_->last_conductivity_us()));
  EXPECT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(WrsNodeTest, UpaBatchTriggersWhenTankFills)
{
  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 30.0));
  node_->configure();
  node_->activate();
  // Enormous urine rate fills the WSTA past the 70% start fraction quickly.
  spin_and_feed(500ms, 3.5e6);
  EXPECT_TRUE(node_->upa_processing());
  // Tank is clamped to its capacity (default 22 kg).
  EXPECT_LE(node_->wastewater_kg(), 22.0 + 1e-6);
}
