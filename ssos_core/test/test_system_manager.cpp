#include <gtest/gtest.h>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ssos_core/system_manager.hpp"
#include "space_station_interfaces/msg/system_state.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/srv/register_subsystem.hpp"

using namespace std::chrono_literals;
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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}