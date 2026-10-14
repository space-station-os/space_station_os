#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ssos_eps/nodes/eps_node.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using ssos_eps::nodes::BCDUStatus;
using ssos_eps::nodes::FaultEvent;
using ssos_eps::nodes::SubsystemHeartbeat;
using ssos_eps::nodes::WorldState;

class EpsNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<ssos_eps::nodes::EpsNode>();
    probe_ = std::make_shared<rclcpp::Node>("eps_test_probe");
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(probe_);
  }

  void recreate_node(const rclcpp::NodeOptions & options)
  {
    executor_.remove_node(node_->get_node_base_interface());
    node_ = std::make_shared<ssos_eps::nodes::EpsNode>(options);
    executor_.add_node(node_->get_node_base_interface());
  }

  void TearDown() override
  {
    executor_.remove_node(probe_);
    executor_.remove_node(node_->get_node_base_interface());
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end && rclcpp::ok()) {
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  void bring_up()
  {
    ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  }

  void tear_down()
  {
    EXPECT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
  }

  std::shared_ptr<ssos_eps::nodes::EpsNode> node_;
  rclcpp::Node::SharedPtr probe_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(EpsNodeTest, ConfigureActivateDeactivateCleanup)
{
  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 30.0));
  bring_up();
  spin_for(300ms);

  const auto & state = node_->last_state();
  EXPECT_EQ(state.orus.size(), 24u);
  EXPECT_EQ(state.channels.size(), 12u);
  EXPECT_GT(state.load_demand_w, 0.0);
  EXPECT_GE(state.battery_soc, 0.0);
  EXPECT_LE(state.battery_soc, 1.0);

  tear_down();
}

TEST_F(EpsNodeTest, FrozenSimulationTimeDoesNotAdvanceBatterySoc)
{
  // Overriding use_sim_time at construction makes ROS time frozen from the
  // first step; setting it afterwards only takes effect asynchronously.
  recreate_node(rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("use_sim_time", true),
      rclcpp::Parameter("step_rate_hz", 30.0),
      rclcpp::Parameter("battery.initial_soc", 0.50)}));
  bring_up();

  // No /clock is published, so ROS time stays frozen while wall timers fire.
  spin_for(300ms);
  EXPECT_TRUE(node_->get_clock()->ros_time_is_active());
  EXPECT_DOUBLE_EQ(node_->last_state().battery_soc, 0.50);
  const double soc_before = node_->last_state().battery_soc;
  spin_for(300ms);
  EXPECT_DOUBLE_EQ(node_->last_state().battery_soc, soc_before);

  tear_down();
}

TEST_F(EpsNodeTest, PublishesBcduBatteryAndHeartbeatTelemetry)
{
  std::vector<BCDUStatus> bcdu;
  std::vector<sensor_msgs::msg::BatteryState> oru;
  std::vector<SubsystemHeartbeat> heartbeats;
  auto bcdu_sub = probe_->create_subscription<BCDUStatus>(
    "/ssos/eps/bcdu/status", 10, [&](BCDUStatus::SharedPtr m) {bcdu.push_back(*m);});
  auto oru_sub = probe_->create_subscription<sensor_msgs::msg::BatteryState>(
    "/ssos/eps/battery/oru_0/state", 10,
    [&](sensor_msgs::msg::BatteryState::SharedPtr m) {oru.push_back(*m);});
  auto hb_sub = probe_->create_subscription<SubsystemHeartbeat>(
    "/ssos/eps/heartbeat", 10,
    [&](SubsystemHeartbeat::SharedPtr m) {heartbeats.push_back(*m);});

  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 20.0));
  bring_up();
  spin_for(600ms);

  ASSERT_FALSE(bcdu.empty());
  ASSERT_FALSE(oru.empty());
  ASSERT_FALSE(heartbeats.empty());
  EXPECT_EQ(heartbeats.back().subsystem_name, "eps");
  EXPECT_EQ(heartbeats.back().lifecycle_state, SubsystemHeartbeat::LIFECYCLE_ACTIVE);
  EXPECT_TRUE(heartbeats.back().healthy);
  EXPECT_EQ(oru.back().cell_voltage.size(), 30u);
  EXPECT_GT(oru.back().voltage, 0.0f);
  EXPECT_EQ(bcdu.back().header.frame_id, "bcdu");

  tear_down();
}

TEST_F(EpsNodeTest, EclipseFromWorldStateSwitchesBcdusToDischarge)
{
  auto world_pub = probe_->create_publisher<WorldState>("/sim/world_state", 10);
  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 20.0));
  bring_up();

  WorldState sun;
  sun.solar_flux_w_m2 = 1361.0;
  sun.in_eclipse = false;
  for (int i = 0; i < 10; ++i) {
    world_pub->publish(sun);
    spin_for(30ms);
  }
  EXPECT_GT(node_->last_state().generation_w, 0.0);
  EXPECT_EQ(node_->last_state().channels.front().mode, ssos_eps::bcdu::BcduMode::CHARGING);

  WorldState shadow;
  shadow.solar_flux_w_m2 = 0.0;
  shadow.in_eclipse = true;
  for (int i = 0; i < 10; ++i) {
    world_pub->publish(shadow);
    spin_for(30ms);
  }
  EXPECT_DOUBLE_EQ(node_->last_state().generation_w, 0.0);
  EXPECT_EQ(node_->last_state().channels.front().mode,
            ssos_eps::bcdu::BcduMode::DISCHARGING);

  tear_down();
}

TEST_F(EpsNodeTest, LowSocFaultIsPublishedOnceOnTransition)
{
  int low_soc_faults = 0;
  auto fault_sub = probe_->create_subscription<FaultEvent>(
    "/ssos/fault_event", 10, [&](FaultEvent::SharedPtr m) {
      if (m->subsystem_name == "eps" && m->fault_type == "low_battery_soc") {
        ++low_soc_faults;
      }
    });

  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 30.0));
  node_->set_parameter(rclcpp::Parameter("battery.initial_soc", 0.10));
  bring_up();
  spin_for(500ms);

  EXPECT_FALSE(node_->last_state().healthy);
  EXPECT_EQ(low_soc_faults, 1);

  tear_down();
}

TEST_F(EpsNodeTest, FaultHookFailsOruLive)
{
  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 30.0));
  bring_up();
  spin_for(100ms);
  EXPECT_TRUE(node_->last_state().healthy);

  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("faults.failed_oru", 3)).successful);
  spin_for(200ms);
  EXPECT_TRUE(node_->last_state().oru_failed);
  EXPECT_EQ(node_->last_state().orus[3].health, ssos_eps::battery::BatteryHealth::FAILED);

  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("faults.failed_oru", -1)).successful);
  spin_for(200ms);
  EXPECT_FALSE(node_->last_state().oru_failed);

  tear_down();
}

TEST_F(EpsNodeTest, RejectsInvalidLiveParameter)
{
  bring_up();
  EXPECT_FALSE(node_->set_parameter(
      rclcpp::Parameter("bcdu.charge_efficiency", 1.5)).successful);
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("load_demand_w", -1.0)).successful);
  EXPECT_TRUE(node_->set_parameter(
      rclcpp::Parameter("solar.array.degradation_factor", 0.5)).successful);
  tear_down();
}
