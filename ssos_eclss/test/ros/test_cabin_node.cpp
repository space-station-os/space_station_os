#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "ssos_eclss/nodes/cabin_node.hpp"

using namespace ssos_eclss::nodes;
using namespace std::chrono_literals;
using std_msgs::msg::Float64;

class CabinNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<CabinNode>();
    feeder_ = std::make_shared<rclcpp::Node>("cabin_test_feeder");
    crew_co2_ = feeder_->create_publisher<Float64>("/ssos/crew/co2_kg_day", 10);
    ogs_o2_ = feeder_->create_publisher<Float64>("/ssos/ogs/o2_kg_day", 10);
  }

  void spin_and_feed(std::chrono::milliseconds d)
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_->get_node_base_interface());
    exec.add_node(feeder_);
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end && rclcpp::ok()) {
      Float64 m;
      m.data = 4.0; crew_co2_->publish(m);
      m.data = 5.3; ogs_o2_->publish(m);
      exec.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  std::shared_ptr<CabinNode> node_;
  rclcpp::Node::SharedPtr feeder_;
  rclcpp::Publisher<Float64>::SharedPtr crew_co2_;
  rclcpp::Publisher<Float64>::SharedPtr ogs_o2_;
};

TEST_F(CabinNodeTest, LifecycleAndAtmosphereTelemetry)
{
  using lifecycle_msgs::msg::State;
  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 30.0));
  EXPECT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  spin_and_feed(400ms);
  // Atmosphere initialises to a nominal ISS-like ppCO2 and stays finite.
  const double ppm = node_->last_co2_ppm();
  EXPECT_GT(ppm, 0.0);
  EXPECT_LT(ppm, 1.0e5);
  EXPECT_TRUE(std::isfinite(ppm));
  EXPECT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(CabinNodeTest, EnableAutoFaultsAppliesLive)
{
  node_->configure();
  const auto results =
    node_->set_parameters({rclcpp::Parameter("enable_auto_faults", true)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_TRUE(results[0].successful);
}
