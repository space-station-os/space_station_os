#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "space_station_eclss/ogs_system.hpp"
#include "space_station_eclss/action/oxygen_generation.hpp"
#include "space_station_eclss/srv/request_product_water.hpp"
#include "space_station_eclss/srv/co2_request.hpp"
#include "space_station_eclss/srv/grey_water.hpp"

using OxygenGeneration = space_station_eclss::action::OxygenGeneration;
using GoalHandleOGS = rclcpp_action::ClientGoalHandle<OxygenGeneration>;

class OGSTestFixture : public ::testing::Test {
protected:
  static void SetUpTestCase() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    // Main OGS node
    ogs_node_ = std::make_shared<space_station_eclss::OGSSystem>();

    // Test client node
    client_node_ = std::make_shared<rclcpp::Node>("ogs_test_client");

    // === Mock WRS (Product Water) ===
    wrs_mock_ = client_node_->create_service<space_station_eclss::srv::RequestProductWater>(
        "/wrs/product_water_request",
        [](const std::shared_ptr<space_station_eclss::srv::RequestProductWater::Request> req,
           std::shared_ptr<space_station_eclss::srv::RequestProductWater::Response> res) {
          RCLCPP_INFO(rclcpp::get_logger("ogs_test_node"),
                      "[MOCK WRS] Received request for %.2f L water", req->amount);
          res->success = true;
          res->water_granted = req->amount;
          res->message = "Mock water granted";
        });

    // === Mock ARS (CO₂) ===
    ars_mock_ = client_node_->create_service<space_station_eclss::srv::Co2Request>(
        "/ars/request_co2",
        [](const std::shared_ptr<space_station_eclss::srv::Co2Request::Request> req,
           std::shared_ptr<space_station_eclss::srv::Co2Request::Response> res) {
          RCLCPP_INFO(rclcpp::get_logger("ogs_test_node"),
                      "[MOCK ARS] Received request for %.2f g CO₂", req->co2_req);
          res->success = true;
          res->co2_resp = req->co2_req;
          res->message = "Mock CO₂ granted";
        });

    // === Mock Grey Water ===
    grey_mock_ = client_node_->create_service<space_station_eclss::srv::GreyWater>(
        "/grey_water",
        [](const std::shared_ptr<space_station_eclss::srv::GreyWater::Request> req,
           std::shared_ptr<space_station_eclss::srv::GreyWater::Response> res) {
          RCLCPP_INFO(rclcpp::get_logger("ogs_test_node"),
                      "[MOCK GreyWater] Received %.2f L grey water", req->gray_water_liters);
          res->success = true;
          res->message = "Mock grey water accepted";
        });

    // Executor
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(ogs_node_);
   
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  std::shared_ptr<space_station_eclss::OGSSystem> ogs_node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<rclcpp::Executor> executor_;
  std::thread spin_thread_;

  rclcpp::Service<space_station_eclss::srv::RequestProductWater>::SharedPtr wrs_mock_;
  rclcpp::Service<space_station_eclss::srv::Co2Request>::SharedPtr ars_mock_;
  rclcpp::Service<space_station_eclss::srv::GreyWater>::SharedPtr grey_mock_;
};

TEST_F(OGSTestFixture, ActionGoalCompletes) {
  auto action_client = rclcpp_action::create_client<OxygenGeneration>(
    client_node_, "oxygen_generation");
  ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(5)));

  OxygenGeneration::Goal goal;
  goal.input_water_mass = 100.0;
  goal.iodine_concentration = 0.1;

  auto send_future = action_client->async_send_goal(goal);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node_, send_future, std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = send_future.get();
  ASSERT_TRUE(goal_handle);

  auto result_future = action_client->async_get_result(goal_handle);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node_, result_future, std::chrono::seconds(15)),
    rclcpp::FutureReturnCode::SUCCESS);

  auto result = result_future.get().result;
  EXPECT_TRUE(result->success);
  EXPECT_GT(result->total_o2_generated, 0.0);
}
