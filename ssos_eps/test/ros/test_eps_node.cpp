#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ssos_eps/nodes/eps_node.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using ssos_eps::nodes::BCDUStatus;
using ssos_eps::nodes::DDCUStatus;
using ssos_eps::nodes::FaultEvent;
using ssos_eps::nodes::Load;
using ssos_eps::nodes::MBSUStatus;
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
  EXPECT_FALSE(node_->set_parameter(
      rclcpp::Parameter("loads.demand_scale", -1.0)).successful);
  EXPECT_TRUE(node_->set_parameter(
      rclcpp::Parameter("solar.array.degradation_factor", 0.5)).successful);
  tear_down();
}

TEST_F(EpsNodeTest, OruOverTemperatureReachesHeartbeatAndFaultEvent)
{
  int overtemp_faults = 0;
  std::vector<SubsystemHeartbeat> heartbeats;
  auto fault_sub = probe_->create_subscription<FaultEvent>(
    "/ssos/fault_event", 10, [&](FaultEvent::SharedPtr m) {
      if (m->subsystem_name == "eps" && m->fault_type == "battery_oru_overtemp") {
        ++overtemp_faults;
      }
    });
  auto hb_sub = probe_->create_subscription<SubsystemHeartbeat>(
    "/ssos/eps/heartbeat", 10,
    [&](SubsystemHeartbeat::SharedPtr m) {heartbeats.push_back(*m);});

  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 30.0));
  bring_up();
  spin_for(200ms);
  ASSERT_FALSE(heartbeats.empty());
  EXPECT_TRUE(heartbeats.back().healthy);

  // Same reproduction as review: limit below the ORU temperature (~10 C).
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("battery.max_temp_c", 5.0)).successful);
  spin_for(300ms);
  EXPECT_TRUE(node_->last_state().oru_overheat);
  EXPECT_FALSE(heartbeats.back().healthy);
  EXPECT_EQ(heartbeats.back().status_message, "battery ORU over temperature");
  EXPECT_EQ(overtemp_faults, 1);

  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("battery.max_temp_c", 35.0)).successful);
  spin_for(300ms);
  EXPECT_TRUE(heartbeats.back().healthy);

  tear_down();
}

TEST_F(EpsNodeTest, LoadRequestServiceAdmitsAndReleases)
{
  auto client = probe_->create_client<Load>("/ssos/eps/load_request");
  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 30.0));
  bring_up();
  spin_for(100ms);
  ASSERT_TRUE(client->wait_for_service(1s));

  auto call = [&](const std::string & id, uint8_t action, double power, int ddcu) {
      auto req = std::make_shared<Load::Request>();
      req->load_id = id;
      req->action = action;
      req->power_w = power;
      req->priority = 2;
      req->ddcu_index = ddcu;
      auto future = client->async_send_request(req);
      EXPECT_EQ(executor_.spin_until_future_complete(future, 2s),
                rclcpp::FutureReturnCode::SUCCESS);
      return future.get();
    };

  const auto granted = call("experiment", Load::Request::ACTION_REQUEST, 1500.0, -1);
  ASSERT_TRUE(granted->success) << granted->message;
  EXPECT_GE(granted->ddcu_index, 0);
  EXPECT_DOUBLE_EQ(granted->granted_power_w, 1500.0);
  EXPECT_GT(granted->supply_voltage, 123.0);

  spin_for(200ms);
  bool served = false;
  for (const auto & l : node_->last_state().loads) {
    if (l.id == "experiment") {
      served = l.served_w > 1499.0;
    }
  }
  EXPECT_TRUE(served);

  EXPECT_TRUE(call("experiment", Load::Request::ACTION_RELEASE, 0.0, -1)->success);
  EXPECT_FALSE(call("experiment", Load::Request::ACTION_RELEASE, 0.0, -1)->success);

  const auto empty = call("", Load::Request::ACTION_REQUEST, 100.0, -1);
  EXPECT_FALSE(empty->success);
  EXPECT_EQ(empty->ddcu_index, -1);
  EXPECT_FALSE(call("neg", Load::Request::ACTION_REQUEST, -5.0, -1)->success);
  EXPECT_FALSE(call("bad_ddcu", Load::Request::ACTION_REQUEST, 100.0, 50)->success);
  EXPECT_FALSE(call("too_big", Load::Request::ACTION_REQUEST, 7000.0, -1)->success);
  EXPECT_FALSE(call("bad_action", 7, 100.0, -1)->success);

  tear_down();
}

TEST_F(EpsNodeTest, PublishesMbsuAndDdcuStatus)
{
  std::vector<MBSUStatus> mbsu;
  std::vector<DDCUStatus> ddcu;
  auto mbsu_sub = probe_->create_subscription<MBSUStatus>(
    "/ssos/eps/mbsu/mbsu_0/status", 10, [&](MBSUStatus::SharedPtr m) {mbsu.push_back(*m);});
  auto ddcu_sub = probe_->create_subscription<DDCUStatus>(
    "/ssos/eps/ddcu/ddcu_0/status", 10, [&](DDCUStatus::SharedPtr m) {ddcu.push_back(*m);});

  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 20.0));
  bring_up();
  spin_for(500ms);

  ASSERT_FALSE(mbsu.empty());
  ASSERT_FALSE(ddcu.empty());
  EXPECT_EQ(mbsu.back().source_channels, (std::vector<int32_t>{0, 1, 2}));
  EXPECT_EQ(mbsu.back().available_sources, 3);
  EXPECT_FALSE(mbsu.back().unstable);
  EXPECT_GT(mbsu.back().supplied_power, 0.0);
  EXPECT_EQ(ddcu.back().mode, "regulating");
  EXPECT_EQ(ddcu.back().mbsu_index, 0);
  EXPECT_DOUBLE_EQ(ddcu.back().rated_power, 6250.0);
  EXPECT_GT(ddcu.back().output_power, 0.0);
  EXPECT_GT(ddcu.back().active_loads, 0);
  EXPECT_FALSE(ddcu.back().fault);

  tear_down();
}

TEST_F(EpsNodeTest, DdcuFaultHookDropsLoadsAndReportsOnce)
{
  int ddcu_faults = 0;
  auto fault_sub = probe_->create_subscription<FaultEvent>(
    "/ssos/fault_event", 10, [&](FaultEvent::SharedPtr m) {
      if (m->subsystem_name == "eps" && m->fault_type == "ddcu_fault") {
        ++ddcu_faults;
      }
    });

  node_->set_parameter(rclcpp::Parameter("step_rate_hz", 30.0));
  bring_up();
  spin_for(100ms);
  EXPECT_TRUE(node_->last_state().healthy);

  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("faults.faulted_ddcu", 0)).successful);
  spin_for(300ms);
  EXPECT_TRUE(node_->last_state().ddcu_fault);
  EXPECT_TRUE(node_->last_state().load_shed);
  EXPECT_EQ(ddcu_faults, 1);

  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("faults.faulted_ddcu", -1)).successful);
  spin_for(200ms);
  EXPECT_TRUE(node_->last_state().healthy);

  tear_down();
}
