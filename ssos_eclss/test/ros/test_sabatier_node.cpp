#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "ssos_eclss/nodes/sabatier_node.hpp"

using namespace ssos_eclss::nodes;
using namespace std::chrono_literals;
using std_msgs::msg::Float64;

class SabatierNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<SabatierNode>();
    feeder_ = std::make_shared<rclcpp::Node>("sab_test_feeder");
    co2_ = feeder_->create_publisher<Float64>("/ssos/ars/co2_removal_kg_day", 10);
    o2_ = feeder_->create_publisher<Float64>("/ssos/ogs/o2_kg_day", 10);
  }

  void spin_and_feed(std::chrono::milliseconds d)
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_->get_node_base_interface());
    exec.add_node(feeder_);
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end && rclcpp::ok()) {
      Float64 m;
      m.data = 4.16; co2_->publish(m);   // ARS-desorbed CO2
      m.data = 5.3; o2_->publish(m);     // OGS O2 -> H2 = 2x
      exec.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  std::shared_ptr<SabatierNode> node_;
  rclcpp::Node::SharedPtr feeder_;
  rclcpp::Publisher<Float64>::SharedPtr co2_;
  rclcpp::Publisher<Float64>::SharedPtr o2_;
};

TEST_F(SabatierNodeTest, HoldsTemperatureAndMakesWater)
{
  using lifecycle_msgs::msg::State;
  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 30.0));
  EXPECT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  spin_and_feed(400ms);
  const auto & r = node_->last_result();
  EXPECT_NEAR(r.reactor_temp_k, 648.0, 25.0);  // trim heater holds the setpoint
  EXPECT_GT(r.conversion, 0.9);
  EXPECT_GT(r.water_produced_kg_s, 0.0);        // water routed to the WRS
  EXPECT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}
