#include <gtest/gtest.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ssos_core/system_manager.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/system_state.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/srv/register_subsystem.hpp"

using namespace std::chrono_literals;
using FaultEvent = space_station_interfaces::msg::FaultEvent;
using SystemState = space_station_interfaces::msg::SystemState;
using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using RegisterSubsystem = space_station_interfaces::srv::RegisterSubsystem;

class SystemManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    manager_ = std::make_shared<ssos_core::SystemManager>();

    manager_->configure();
    manager_->activate();

    client_ = std::make_shared<rclcpp::Node>("test_client");
    register_client_ = client_->create_client<RegisterSubsystem>("/ssos/register_subsystem");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(manager_->get_node_base_interface());
    executor_->add_node(client_);
  }

  void TearDown() override
  {
    manager_.reset();
    client_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some(10ms);
    }
  }

  bool register_subsystem(const std::string & name, const std::string & heartbeat_topic)
  {
    auto req = std::make_shared<RegisterSubsystem::Request>();
    req->subsystem_name = name;
    req->heartbeat_topic = heartbeat_topic;

    auto future = register_client_->async_send_request(req);
    spin_for(500ms);

    if (future.wait_for(0s) == std::future_status::ready) {
      return future.get()->accepted;
    }
    return false;
  }

  std::shared_ptr<ssos_core::SystemManager> manager_;
  std::shared_ptr<rclcpp::Node> client_;
  rclcpp::Client<RegisterSubsystem>::SharedPtr register_client_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(SystemManagerTest, StartsInInitState)
{
  EXPECT_EQ(manager_->get_current_state(), SystemState::INIT);
}

TEST_F(SystemManagerTest, StaysInitWithNoSubsystems)
{
  spin_for(200ms);
  EXPECT_EQ(manager_->evaluate_system_state(), SystemState::INIT);
}

TEST_F(SystemManagerTest, RegisterSubsystemAccepted)
{
  ASSERT_TRUE(register_client_->wait_for_service(2s));
  bool accepted = register_subsystem("eclss", "/ssos/eclss/heartbeat");
  EXPECT_TRUE(accepted);
}

TEST_F(SystemManagerTest, DegradedWhenHeartbeatMissing)
{
  ASSERT_TRUE(register_client_->wait_for_service(2s));
  register_subsystem("eclss", "/ssos/eclss/heartbeat");
  spin_for(200ms);
  EXPECT_EQ(manager_->get_current_state(), SystemState::DEGRADED);
}

TEST_F(SystemManagerTest, NominalWhenAllHealthy)
{
  ASSERT_TRUE(register_client_->wait_for_service(2s));
  register_subsystem("eclss", "/ssos/eclss/heartbeat");

  auto hb_pub = client_->create_publisher<SubsystemHeartbeat>("/ssos/eclss/heartbeat", 10);

  auto hb = SubsystemHeartbeat();
  hb.subsystem_name = "eclss";
  hb.lifecycle_state = SubsystemHeartbeat::LIFECYCLE_ACTIVE;
  hb.healthy = true;
  hb.status_message = "OK";

  spin_for(200ms);
  hb_pub->publish(hb);
  spin_for(500ms);

  EXPECT_EQ(manager_->get_current_state(), SystemState::NOMINAL);
}

class SystemManagerRecoveryTest : public SystemManagerTest
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    manager_ = std::make_shared<ssos_core::SystemManager>(
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("recovery_hold_s", recovery_hold_s()),
        rclcpp::Parameter("eval_period_s", 0.1)}));
    manager_->configure();
    manager_->activate();

    client_ = std::make_shared<rclcpp::Node>("test_client");
    register_client_ = client_->create_client<RegisterSubsystem>("/ssos/register_subsystem");
    hb_pub_ = client_->create_publisher<SubsystemHeartbeat>("/ssos/eps/heartbeat", 10);
    fault_pub_ = client_->create_publisher<FaultEvent>("/ssos/fault_event", 10);
    state_sub_ = client_->create_subscription<SystemState>(
      "/ssos/system_state", 10,
      [this](SystemState::SharedPtr m) {states_.push_back(m->state);});

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(manager_->get_node_base_interface());
    executor_->add_node(client_);

    ASSERT_TRUE(register_client_->wait_for_service(2s));
    ASSERT_TRUE(register_subsystem("eps", "/ssos/eps/heartbeat"));
  }

  void TearDown() override
  {
    state_sub_.reset();
    hb_pub_.reset();
    fault_pub_.reset();
    SystemManagerTest::TearDown();
  }

  virtual double recovery_hold_s() const {return 0.5;}

  // Publish heartbeats at 20 Hz for the given duration.
  void heartbeats_for(std::chrono::milliseconds duration, bool healthy)
  {
    SubsystemHeartbeat hb;
    hb.subsystem_name = "eps";
    hb.lifecycle_state = SubsystemHeartbeat::LIFECYCLE_ACTIVE;
    hb.healthy = healthy;
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      hb_pub_->publish(hb);
      spin_for(50ms);
    }
  }

  // Publish heartbeats until the manager reaches `state` or the timeout runs
  // out. Waiting on the outcome keeps the tests robust on a loaded CI runner.
  bool heartbeats_until(uint8_t state, std::chrono::milliseconds timeout, bool healthy = true)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      heartbeats_for(50ms, healthy);
      if (manager_->get_current_state() == state) {
        return true;
      }
    }
    return false;
  }

  void inject_fault(uint8_t severity = FaultEvent::SEVERITY_CRITICAL)
  {
    FaultEvent ev;
    ev.subsystem_name = "eps";
    ev.fault_type = "bcdu_fault";
    ev.severity = severity;
    ev.description = "test fault";
    fault_pub_->publish(ev);
  }

  bool saw(uint8_t state) const
  {
    return std::find(states_.begin(), states_.end(), state) != states_.end();
  }

  rclcpp::Publisher<SubsystemHeartbeat>::SharedPtr hb_pub_;
  rclcpp::Publisher<FaultEvent>::SharedPtr fault_pub_;
  rclcpp::Subscription<SystemState>::SharedPtr state_sub_;
  std::vector<uint8_t> states_;
};

TEST_F(SystemManagerRecoveryTest, FaultClearsAfterHealthyHold)
{
  ASSERT_TRUE(heartbeats_until(SystemState::NOMINAL, 5s));

  inject_fault();
  heartbeats_for(200ms, true);
  EXPECT_EQ(manager_->get_current_state(), SystemState::DEGRADED);

  // 0.5 s hold, then RECOVERY and NOMINAL on the following evaluations.
  EXPECT_TRUE(heartbeats_until(SystemState::NOMINAL, 5s));
  EXPECT_TRUE(saw(SystemState::DEGRADED));
  EXPECT_TRUE(saw(SystemState::RECOVERY));

  // RECOVERY comes between DEGRADED and the final NOMINAL.
  const auto degraded = std::find(states_.begin(), states_.end(), SystemState::DEGRADED);
  const auto recovery = std::find(degraded, states_.end(), SystemState::RECOVERY);
  ASSERT_NE(recovery, states_.end());
  EXPECT_NE(std::find(recovery, states_.end(), SystemState::NOMINAL), states_.end());
}

TEST_F(SystemManagerRecoveryTest, FaultStaysWhileSubsystemUnhealthy)
{
  ASSERT_TRUE(heartbeats_until(SystemState::NOMINAL, 5s));
  inject_fault();
  heartbeats_for(1200ms, false);
  EXPECT_EQ(manager_->get_current_state(), SystemState::DEGRADED);
  EXPECT_FALSE(saw(SystemState::RECOVERY));

  // Once healthy again it still has to hold healthy before recovering.
  heartbeats_for(200ms, true);
  EXPECT_EQ(manager_->get_current_state(), SystemState::DEGRADED);
  EXPECT_TRUE(heartbeats_until(SystemState::NOMINAL, 5s));
}

TEST_F(SystemManagerRecoveryTest, ClearFaultsCommandIsNotAFault)
{
  ASSERT_TRUE(heartbeats_until(SystemState::NOMINAL, 5s));
  // Registration passes through DEGRADED until the first heartbeat; only
  // look at what happens after the command.
  states_.clear();
  FaultEvent ev;
  ev.subsystem_name = "eps";
  ev.fault_type = "clear_faults";
  ev.severity = FaultEvent::SEVERITY_CRITICAL;
  fault_pub_->publish(ev);
  heartbeats_for(300ms, true);
  EXPECT_EQ(manager_->get_current_state(), SystemState::NOMINAL);
  EXPECT_FALSE(saw(SystemState::DEGRADED));
}

TEST_F(SystemManagerRecoveryTest, SafeStaysLatched)
{
  ASSERT_TRUE(heartbeats_until(SystemState::NOMINAL, 5s));
  inject_fault(FaultEvent::SEVERITY_EMERGENCY);
  heartbeats_for(1500ms, true);
  EXPECT_EQ(manager_->get_current_state(), SystemState::SAFE);
  EXPECT_FALSE(saw(SystemState::RECOVERY));
}

class SystemManagerNoRecoveryTest : public SystemManagerRecoveryTest
{
protected:
  double recovery_hold_s() const override {return -1.0;}
};

TEST_F(SystemManagerNoRecoveryTest, NegativeHoldKeepsFaultLatched)
{
  ASSERT_TRUE(heartbeats_until(SystemState::NOMINAL, 5s));
  inject_fault();
  heartbeats_for(1500ms, true);
  EXPECT_EQ(manager_->get_current_state(), SystemState::DEGRADED);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}