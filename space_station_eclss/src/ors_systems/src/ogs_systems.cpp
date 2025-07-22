#include "space_station_eclss/ogs_system.hpp"

OGSSystem::OGSSystem() : Node("ogs_system")
{
  this->declare_parameter("enable_failure", false);
  enable_failure_ = this->get_parameter("enable_failure").as_bool();

  this->declare_parameter("electrolysis_temp", 100.0);
  electrolysis_temp_ = this->get_parameter("electrolysis_temp").as_double();

  this->declare_parameter("o2_efficiency", 0.95);
  o2_efficiency_ = this->get_parameter("o2_efficiency").as_double();

  this->declare_parameter("sabatier_efficiency", 0.75);
  sabatier_efficiency_ = this->get_parameter("sabatier_efficiency").as_double();

  this->declare_parameter("sabatier_temp", 300.0);
  sabatier_temp_ = this->get_parameter("sabatier_temp").as_double();

  this->declare_parameter("sabatier_pressure", 1.0);
  sabatier_pressure_ = this->get_parameter("sabatier_pressure").as_double();

  this->declare_parameter("min_o2_capacity", 100.0);
  min_o2_capacity_ = this->get_parameter("min_o2_capacity").as_double();

  this->declare_parameter("max_o2_capacity", 10000.0);
  max_o2_capacity_ = this->get_parameter("max_o2_capacity").as_double();

  // Setup clients, servers, publishers
  action_server_ = rclcpp_action::create_server<space_station_eclss::action::OxygenGeneration>(
    this,
    "oxygen_generation",
    std::bind(&OGSSystem::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&OGSSystem::handle_cancel, this, std::placeholders::_1),
    std::bind(&OGSSystem::execute_goal, this, std::placeholders::_1));

  water_client_ = this->create_client<space_station_eclss::srv::RequestProductWater>("/wrs/product_water_request");
  co2_client_ = this->create_client<space_station_eclss::srv::Co2Request>("/ars/request_co2");

  o2_server_ = this->create_service<space_station_eclss::srv::O2Request>(
    "/ogs/request_o2",
    std::bind(&OGSSystem::o2_service_callback, this, std::placeholders::_1, std::placeholders::_2));

  gray_water_client_ = this->create_client<space_station_eclss::srv::GreyWater>("/grey_water");

  o2_pub_ = this->create_publisher<std_msgs::msg::Float64>("/o2_storage", 10);
  ch4_pub_ = this->create_publisher<std_msgs::msg::Float64>("/methane_vented", 10);
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/ogs/diagnostics", 10);

  timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    std::bind(&OGSSystem::publish_periodic_status, this));

  disable_failure_ = this->create_subscription<std_msgs::msg::Bool>(
    "/ogs/self_diagnosis",
    10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      enable_failure_ = msg->data;
      RCLCPP_INFO(this->get_logger(), "Failure simulation %s", enable_failure_ ? "enabled" : "disabled");
    }
  );

  RCLCPP_INFO(this->get_logger(), "OGS system ready.");
}


rclcpp_action::GoalResponse OGSSystem::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const space_station_eclss::action::OxygenGeneration::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received oxygen generation goal.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OGSSystem::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<space_station_eclss::action::OxygenGeneration>>)
{
  return rclcpp_action::CancelResponse::REJECT;
}

void OGSSystem::execute_goal(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<space_station_eclss::action::OxygenGeneration>> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<space_station_eclss::action::OxygenGeneration::Result>();
  auto feedback = std::make_shared<space_station_eclss::action::OxygenGeneration::Feedback>();

  RCLCPP_INFO(this->get_logger(), "Starting deionization: input water = %.2f L, iodine = %.2f ppm",
              goal->input_water_mass, goal->iodine_concentration);

  // === Step 1: Request Product Water ===
  auto water_req = std::make_shared<space_station_eclss::srv::RequestProductWater::Request>();
  water_req->amount = goal->input_water_mass;

  water_client_->async_send_request(water_req,
    [this, goal_handle, goal, result, feedback](rclcpp::Client<space_station_eclss::srv::RequestProductWater>::SharedFuture water_future)
    {
      auto water_resp = water_future.get();
      if (!water_resp->success) {
        publish_failure_diagnostics("Deionization", water_resp->message);
        result->success = false;
        result->summary_message = "Water request failed: " + water_resp->message;
        goal_handle->abort(result);
        return;
      }

      double water_granted = water_resp->water_granted;
      double o2_generated = o2_efficiency_ * water_granted;
      double h2_generated = (1.0 - o2_efficiency_) * water_granted;
      latest_o2_ += o2_generated;

      // O2 overflow check
      if (enable_failure_ && latest_o2_ > max_o2_capacity_) {
        std::string msg = "O2 storage overflow: " + std::to_string(latest_o2_) + " g";
        RCLCPP_FATAL(this->get_logger(), "%s", msg.c_str());
        publish_failure_diagnostics("O2Storage", msg);
        result->success = false;
        result->summary_message = msg;
        goal_handle->abort(result);
        return;
      }

      latest_o2_ += o2_generated;

      if (o2_generated <= 0.0) {
        publish_failure_diagnostics("Electrolysis", "Zero oxygen generated");
        result->success = false;
        result->summary_message = "Electrolysis failed: No O₂ generated.";
        goal_handle->abort(result);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Electrolysis output: O2 = %.2f g, H2 = %.2f g", o2_generated, h2_generated);

      std_msgs::msg::Float64 o2_msg;
      o2_msg.data = latest_o2_;
      o2_pub_->publish(o2_msg);

      // === Step 2: Request CO₂ ===
      auto co2_req = std::make_shared<space_station_eclss::srv::Co2Request::Request>();
      co2_req->co2_req = h2_generated * 1000.0;  // grams

      co2_client_->async_send_request(co2_req,
        [this, goal_handle, result, feedback, o2_generated, h2_generated, water_granted](rclcpp::Client<space_station_eclss::srv::Co2Request>::SharedFuture co2_future)
        {
          auto co2_resp = co2_future.get();
          if (!co2_resp->success) {
            publish_failure_diagnostics("Sabatier Reactor", co2_resp->message);
            result->success = false;
            result->summary_message = "CO₂ request failed: " + co2_resp->message;
            goal_handle->abort(result);
            return;
          }

          double co2_granted = co2_resp->co2_resp;
          double ch4 = sabatier_efficiency_ * co2_granted;

          double gray_water = 0.8 * water_granted;

          RCLCPP_INFO(this->get_logger(), "Sabatier output: CH4 = %.2f g, Grey water = %.2f L", ch4, gray_water);
          total_ch4_vented_ += ch4;

          std_msgs::msg::Float64 ch4_msg;
          ch4_msg.data = total_ch4_vented_;
          ch4_pub_->publish(ch4_msg);

          // === Step 3: Send grey water ===
          auto gw_req = std::make_shared<space_station_eclss::srv::GreyWater::Request>();
          gw_req->gray_water_liters = gray_water;

          gray_water_client_->async_send_request(gw_req,
            [this, goal_handle, result, feedback, o2_generated, h2_generated, ch4, gray_water](rclcpp::Client<space_station_eclss::srv::GreyWater>::SharedFuture gw_future)
            {
              auto gw_resp = gw_future.get();
              if (!gw_resp->success) {
                publish_failure_diagnostics("Grey Water Dispatch", gw_resp->message);
                result->success = false;
                result->summary_message = "Gray water transfer failed: " + gw_resp->message;
                goal_handle->abort(result);
                return;
              }

              // === Feedback and Success ===
              feedback->time_step = 1;
              feedback->current_temperature = 293.15;
              feedback->o2_generated = o2_generated;
              feedback->h2_generated = h2_generated;
              feedback->ch4_vented = ch4;
              goal_handle->publish_feedback(feedback);

              RCLCPP_INFO(this->get_logger(), "Oxygen generation complete: O2=%.2f g, CH4=%.2f g, Grey water=%.2f L",
                          o2_generated, ch4, gray_water);

              result->success = true;
              result->total_o2_generated = o2_generated;
              result->total_ch4_vented = ch4;
              result->summary_message = "O2 and CH4 generation complete.";
              goal_handle->succeed(result);
            });
        });
    });
}

void OGSSystem::request_product_water(double amount_liters)
{
  auto request = std::make_shared<space_station_eclss::srv::RequestProductWater::Request>();
  request->amount = amount_liters;

  if (!water_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(this->get_logger(), "Product water service not available");
    return;
  }

  auto future = water_client_->async_send_request(
    request,
    [this](rclcpp::Client<space_station_eclss::srv::RequestProductWater>::SharedFuture future_result) {
      auto res = future_result.get();
      if (res->success) {
        RCLCPP_INFO(this->get_logger(), "Water granted: %.2f L", res->water_granted);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Water request failed: %s", res->message.c_str());
      }
    });
}


void OGSSystem::request_co2(double co2_mass_kg)
{
  auto request = std::make_shared<space_station_eclss::srv::Co2Request::Request>();
  request->co2_req = co2_mass_kg;

  if (!co2_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(this->get_logger(), "CO₂ service not available");
    return;
  }

  auto future = co2_client_->async_send_request(
    request,
    [this](rclcpp::Client<space_station_eclss::srv::Co2Request>::SharedFuture future_result) {
      auto res = future_result.get();
      if (res->success) {
        RCLCPP_INFO(this->get_logger(), "CO₂ granted: %.2f g", res->co2_resp);
      } else {
        RCLCPP_ERROR(this->get_logger(), "CO₂ request failed: %s", res->message.c_str());
      }
    });
}


void OGSSystem::send_gray_water(double amount_liters)
{
  auto request = std::make_shared<space_station_eclss::srv::GreyWater::Request>();
  request->gray_water_liters = amount_liters;

  if (!gray_water_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(this->get_logger(), "Gray water service not available");
    return;
  }

  auto future = gray_water_client_->async_send_request(
    request,
    [this](rclcpp::Client<space_station_eclss::srv::GreyWater>::SharedFuture future_result) {
      auto res = future_result.get();
      if (res->success) {
        RCLCPP_INFO(this->get_logger(), "Gray water sent successfully: %s", res->message.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Gray water send failed: %s", res->message.c_str());
      }
    });
}



void OGSSystem::o2_service_callback(
  const std::shared_ptr<space_station_eclss::srv::O2Request::Request> request,
  std::shared_ptr<space_station_eclss::srv::O2Request::Response> response)
{
  double amount = request->o2_req;
  if (latest_o2_ >= amount) {
    latest_o2_ -= amount;

    // O2 underflow check
    if (enable_failure_ && latest_o2_ < min_o2_capacity_) {
      std::string msg = "O2 storage underflow: " + std::to_string(latest_o2_) + " g";
      RCLCPP_FATAL(this->get_logger(), "%s", msg.c_str());
      publish_failure_diagnostics("O2Storage", msg);
    }

    response->o2_resp = amount;
    response->success = true;
    response->message = "O2 request successful.";

    std_msgs::msg::Float64 o2_msg;
    o2_msg.data = latest_o2_;
    o2_pub_->publish(o2_msg);
  }

}

void OGSSystem::publish_failure_diagnostics(const std::string &unit, const std::string &reason)
{
  diagnostic_msgs::msg::DiagnosticStatus diag;
  diag.name = unit;
  diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  diag.message = reason;
  diag_pub_->publish(diag);
}

void OGSSystem::publish_periodic_status()
{ 
  if (enable_failure_) {
    if (latest_o2_ > max_o2_capacity_) {
      publish_failure_diagnostics("O2Storage", "O2 storage over max threshold.");
      RCLCPP_FATAL(this->get_logger(), "O2 storage overflow: %.2f g", latest_o2_);
    } else if (latest_o2_ < min_o2_capacity_) {
      publish_failure_diagnostics("O2Storage", "O2 storage below min threshold.");
      RCLCPP_FATAL(this->get_logger(), "O2 storage underflow: %.2f g", latest_o2_);
    }
  }

  // Publish current O2 storage
  std_msgs::msg::Float64 o2_msg;
  o2_msg.data = latest_o2_;
  o2_pub_->publish(o2_msg);

  // Publish CH4 vented so far
  std_msgs::msg::Float64 ch4_msg;
  ch4_msg.data = total_ch4_vented_;
  ch4_pub_->publish(ch4_msg);

  // Publish diagnostics
  diagnostic_msgs::msg::DiagnosticStatus diag;
  diag.name = "OGSSystem Heartbeat";
  diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  diag.message = "OGS running. Latest O2: " + std::to_string(latest_o2_) + " g";
  diag_pub_->publish(diag);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto ogs_system = std::make_shared<OGSSystem>();
  rclcpp::spin(ogs_system);
 
  rclcpp::shutdown();
  return 0;
}
