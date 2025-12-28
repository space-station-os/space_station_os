#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "space_station_eclss/ars_systems.hpp"
#include "space_station_eclss/action/air_revitalisation.hpp"
#include "space_station_eclss/srv/co2_request.hpp"
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;
using AirRevitalisation = space_station_eclss::action::AirRevitalisation;
using GoalHandleARS = rclcpp_action::ClientGoalHandle<AirRevitalisation>;

class ARSTestFixture : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }
  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() override
  {
    
    ars_node_ = std::make_shared<space_station_eclss::ARSActionServer>(rclcpp::NodeOptions{});
    client_node_ = std::make_shared<rclcpp::Node>("ars_test_client");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(ars_node_);
    

    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable())
      spin_thread_.join();
  }

  std::shared_ptr<space_station_eclss::ARSActionServer> ars_node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<rclcpp::Executor> executor_;
  std::thread spin_thread_;
};

// ----------------- ACTION TESTS -----------------

TEST_F(ARSTestFixture, ActionGoalSucceeds)
{
  auto action_client = rclcpp_action::create_client<AirRevitalisation>(
      client_node_, "air_revitalisation");
  ASSERT_TRUE(action_client->wait_for_action_server(5s));

  AirRevitalisation::Goal goal;
  goal.initial_co2_mass = 5.0;   // small, should succeed
  goal.initial_moisture_content = 1.0;
  goal.initial_contaminants = 0.1;

  auto gh_future = action_client->async_send_goal(goal);
  ASSERT_EQ(rclcpp::spin_until_future_complete(client_node_, gh_future, 5s),
            rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = gh_future.get();
  ASSERT_TRUE(goal_handle);

  auto result_future = action_client->async_get_result(goal_handle);
  ASSERT_EQ(rclcpp::spin_until_future_complete(client_node_, result_future, 15s),
            rclcpp::FutureReturnCode::SUCCESS);

  auto result = result_future.get().result;
  EXPECT_TRUE(result->success);
  EXPECT_GT(result->cycles_completed, 0);
}

TEST_F(ARSTestFixture, ActionGoalFailsOnOverheating)
{
  ars_node_->set_parameter(rclcpp::Parameter("des1_temp_limit", 20.0)); // force fail
  auto action_client = rclcpp_action::create_client<AirRevitalisation>(
      client_node_, "air_revitalisation");
  ASSERT_TRUE(action_client->wait_for_action_server(5s));

  AirRevitalisation::Goal goal;
  goal.initial_co2_mass = 10.0;
  goal.initial_moisture_content = 50.0;  // high → overheats

  auto gh_future = action_client->async_send_goal(goal);
  rclcpp::spin_until_future_complete(client_node_, gh_future, 5s);
  auto goal_handle = gh_future.get();

  auto result_future = action_client->async_get_result(goal_handle);
  rclcpp::spin_until_future_complete(client_node_, result_future, 10s);

  auto result = result_future.get().result;
  EXPECT_FALSE(result->success);
  EXPECT_NE(result->summary_message.find("Bed failure"), std::string::npos);
}

TEST_F(ARSTestFixture, ActionGoalAbortsOnStorageOverflow)
{
  ars_node_->set_parameter(rclcpp::Parameter("max_co2_storage", 1.0));  // tiny limit
  auto action_client = rclcpp_action::create_client<AirRevitalisation>(
      client_node_, "air_revitalisation");
  ASSERT_TRUE(action_client->wait_for_action_server(5s));

  AirRevitalisation::Goal goal;
  goal.initial_co2_mass = 100.0;

  auto gh_future = action_client->async_send_goal(goal);
  rclcpp::spin_until_future_complete(client_node_, gh_future, 5s);
  auto goal_handle = gh_future.get();

  auto result_future = action_client->async_get_result(goal_handle);
  rclcpp::spin_until_future_complete(client_node_, result_future, 10s);

  auto result = result_future.get().result;
  EXPECT_FALSE(result->success);
  EXPECT_NE(result->summary_message.find("Exceeded CO2 storage"), std::string::npos);
}

TEST_F(ARSTestFixture, ActionGoalCanBeCancelled)
{
  auto action_client = rclcpp_action::create_client<AirRevitalisation>(
      client_node_, "air_revitalisation");
  ASSERT_TRUE(action_client->wait_for_action_server(5s));

  AirRevitalisation::Goal goal;
  goal.initial_co2_mass = 100.0;

  auto gh_future = action_client->async_send_goal(goal);
  rclcpp::spin_until_future_complete(client_node_, gh_future, 5s);
  auto goal_handle = gh_future.get();

  ASSERT_TRUE(goal_handle);

  // Cancel immediately
  auto cancel_future = action_client->async_cancel_goal(goal_handle);
  ASSERT_EQ(rclcpp::spin_until_future_complete(client_node_, cancel_future, 5s),
            rclcpp::FutureReturnCode::SUCCESS);

  auto result_future = action_client->async_get_result(goal_handle);
  rclcpp::spin_until_future_complete(client_node_, result_future, 10s);

  auto result_code = result_future.get().code;
  EXPECT_EQ(result_code, rclcpp_action::ResultCode::CANCELED);
}

// ----------------- SERVICE TESTS -----------------

TEST_F(ARSTestFixture, ServiceRejectsInvalidRequest)
{
  auto client = client_node_->create_client<space_station_eclss::srv::Co2Request>("/ars/request_co2");
  ASSERT_TRUE(client->wait_for_service(5s));

  auto req = std::make_shared<space_station_eclss::srv::Co2Request::Request>();
  req->co2_req = -5.0;

  auto fut = client->async_send_request(req);
  rclcpp::spin_until_future_complete(client_node_, fut, 5s);
  auto res = fut.get();

  EXPECT_FALSE(res->success);
  EXPECT_EQ(res->message, "Invalid request amount");
}

TEST_F(ARSTestFixture, ServiceFailsIfInsufficientStorage)
{
  auto client = client_node_->create_client<space_station_eclss::srv::Co2Request>("/ars/request_co2");
  ASSERT_TRUE(client->wait_for_service(5s));

  auto req = std::make_shared<space_station_eclss::srv::Co2Request::Request>();
  req->co2_req = 9999.0;  // bigger than storage

  auto fut = client->async_send_request(req);
  rclcpp::spin_until_future_complete(client_node_, fut, 5s);
  auto res = fut.get();

  EXPECT_FALSE(res->success);
  EXPECT_EQ(res->message, "Insufficient CO2 in storage");
}

TEST_F(ARSTestFixture, ServiceSucceedsWhenEnoughStorage)
{
  ars_node_->set_parameter(rclcpp::Parameter("max_co2_storage", 5000.0));
  ars_node_->set_parameter(rclcpp::Parameter("sim_time", 1));

  auto action_client = rclcpp_action::create_client<AirRevitalisation>(
      client_node_, "air_revitalisation");
  ASSERT_TRUE(action_client->wait_for_action_server(5s));

  AirRevitalisation::Goal goal;
  goal.initial_co2_mass = 200.0;

  auto gh_future = action_client->async_send_goal(goal);
  rclcpp::spin_until_future_complete(client_node_, gh_future, 5s);
  auto goal_handle = gh_future.get();

  auto result_future = action_client->async_get_result(goal_handle);
  rclcpp::spin_until_future_complete(client_node_, result_future, 10s);

  // Now request less than storage
  auto client = client_node_->create_client<space_station_eclss::srv::Co2Request>("/ars/request_co2");
  client->wait_for_service(5s);

  auto req = std::make_shared<space_station_eclss::srv::Co2Request::Request>();
  req->co2_req = 50.0;

  auto fut = client->async_send_request(req);
  rclcpp::spin_until_future_complete(client_node_, fut, 5s);
  auto res = fut.get();

  EXPECT_TRUE(res->success);
  EXPECT_EQ(res->co2_resp, 50.0);
}

// ----------------- DIAGNOSTICS TEST -----------------

TEST_F(ARSTestFixture, FailureSimulationToggle)
{
  auto pub = client_node_->create_publisher<std_msgs::msg::Bool>("/ars/self_diagnosis", 10);

  auto msg = std_msgs::msg::Bool();
  msg.data = false;
  pub->publish(msg);

  rclcpp::sleep_for(500ms);

  
  EXPECT_FALSE(ars_node_->get_failure_enabled());
}
