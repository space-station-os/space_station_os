#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "ssos_eclss/nodes/crew_node.hpp"

using namespace ssos_eclss::nodes;
using namespace std::chrono_literals;

class CrewNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<CrewNode>();
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

  std::shared_ptr<CrewNode> node_;
};

TEST_F(CrewNodeTest, LifecycleAndPublishesMetabolicLoads)
{
  using lifecycle_msgs::msg::State;
  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 30.0));
  EXPECT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  spin_for(400ms);
  // The crew always produce CO2, consume O2 and demand water.
  EXPECT_GT(node_->last_outputs().co2_kg_s, 0.0);
  EXPECT_GT(node_->last_outputs().o2_consumption_kg_s, 0.0);
  EXPECT_GT(node_->last_outputs().potable_demand_kg_s, 0.0);
  EXPECT_FALSE(node_->last_outputs().activity.empty());
  EXPECT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(CrewNodeTest, CrewSizeParameterAppliesLive)
{
  node_->configure();
  const auto results =
    node_->set_parameters({rclcpp::Parameter("crew_size", 6)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_TRUE(results[0].successful);
}
