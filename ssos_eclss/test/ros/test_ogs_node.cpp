#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "ssos_eclss/nodes/ogs_node.hpp"

using namespace ssos_eclss::nodes;
using namespace std::chrono_literals;
using std_msgs::msg::Float64;

class OgsNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<OgsNode>();
    feeder_ = std::make_shared<rclcpp::Node>("ogs_test_feeder");
    potable_ = feeder_->create_publisher<Float64>("/ssos/wrs/potable_available_kg", 10);
  }

  // feed_kg < 0 -> publish nothing (starved).
  void spin_and_feed(std::chrono::milliseconds d, double feed_kg)
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_->get_node_base_interface());
    exec.add_node(feeder_);
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end && rclcpp::ok()) {
      if (feed_kg >= 0.0) {
        Float64 m;
        m.data = feed_kg;
        potable_->publish(m);
      }
      exec.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  std::shared_ptr<OgsNode> node_;
  rclcpp::Node::SharedPtr feeder_;
  rclcpp::Publisher<Float64>::SharedPtr potable_;
};

TEST_F(OgsNodeTest, ProducesOxygenWhenFed)
{
  using lifecycle_msgs::msg::State;
  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 30.0));
  EXPECT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  spin_and_feed(400ms, 300.0);  // ample feedwater
  EXPECT_GT(node_->last_o2_kg_day(), 0.0);
  EXPECT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(OgsNodeTest, FeedwaterLimitedWhenDry)
{
  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 30.0));
  node_->configure();
  node_->activate();
  spin_and_feed(400ms, -1.0);  // never publish feedwater -> tank stays empty
  EXPECT_NEAR(node_->last_o2_kg_day(), 0.0, 1.0e-6);
}
