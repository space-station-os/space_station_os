#include "space_station_eclss/wrs_systems.hpp"

#include <chrono>
#include <thread>
#include <cstdlib>

using namespace std::chrono_literals;

namespace space_station_eclss
{

WRSActionServer::WRSActionServer(const rclcpp::NodeOptions & options)
: Node("wrs_action_server", options),
  product_water_reserve_(1760.0),
  waste_collector_current_(0.0)
{
  enable_failure_ = this->declare_parameter("enable_failure", true);
  product_water_capacity_ = this->declare_parameter("product_max_capacity", 2000.0f);
  waste_collector_capacity_ = this->declare_parameter("waste_max_capacity", 2000.0f);
  min_product_water_capacity_ = this->declare_parameter("product_min_capacity", 300.0f);

  upa_valve_pressure_ = this->declare_parameter("upa_valve_pressure", 100.0f);
  ionization_valve_pressure_ = this->declare_parameter("ionization_valve_pressure", 90.0f);
  filter_valve_pressure_ = this->declare_parameter("filter_valve_pressure", 85.0f);
  catalytic_valve_pressure_ = this->declare_parameter("catalytic_valve_pressure", 95.0f);

  upa_max_temperature_ = this->declare_parameter("upa_max_temperature", 70.0f);
  ionization_max_temperature_ = this->declare_parameter("ionization_max_temperature", 65.0f);
  filter_max_temperature_ = this->declare_parameter("filter_max_temperature", 60.0f);
  catalytic_max_temperature_ = this->declare_parameter("catalytic_max_temperature", 80.0f);

  action_server_ = rclcpp_action::create_server<WRS>(
    this,
    "water_recovery_systems",
    std::bind(&WRSActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&WRSActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&WRSActionServer::handle_accepted, this, std::placeholders::_1)
  );

  ogs_client_ = rclcpp_action::create_client<space_station_eclss::action::OxygenGeneration>(this, "oxygen_generation");

  water_request_server_ = this->create_service<space_station_eclss::srv::RequestProductWater>(
    "wrs/product_water_request",
    std::bind(&WRSActionServer::handle_product_water_request, this, std::placeholders::_1, std::placeholders::_2)
  );

  gray_water_service_ = this->create_service<space_station_eclss::srv::GreyWater>(
    "/grey_water",
    std::bind(&WRSActionServer::handle_gray_water_request, this, std::placeholders::_1, std::placeholders::_2)
  );

  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("wrs/diagnostics", 10);
  reserve_pub_ = this->create_publisher<std_msgs::msg::Float64>("wrs/product_water_reserve", 10);
  reserve_timer_ = this->create_wall_timer(2s, std::bind(&WRSActionServer::publish_reserve, this));


  disable_failure_ = this->create_subscription<std_msgs::msg::Bool>(
    "/wrs/self_diagnosis",
    10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      enable_failure_ = msg->data;
      RCLCPP_INFO(this->get_logger(), "Failure simulation %s", enable_failure_ ? "enabled" : "disabled");
    }
  );
  RCLCPP_INFO(this->get_logger(), "WRS Action Server initialized");
}

rclcpp_action::GoalResponse WRSActionServer::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const WRS::Goal> goal)
{
  return goal->urine_volume <= 0.0f
    ? rclcpp_action::GoalResponse::REJECT
    : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WRSActionServer::handle_cancel(const std::shared_ptr<GoalHandleWRS>)
{
  RCLCPP_INFO(this->get_logger(), "Cancel request received.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WRSActionServer::handle_accepted(const std::shared_ptr<GoalHandleWRS> goal_handle)
{
  std::thread{std::bind(&WRSActionServer::execute, this, goal_handle)}.detach();
}
void WRSActionServer::execute(const std::shared_ptr<GoalHandleWRS> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<WRS::Feedback>();
  auto result = std::make_shared<WRS::Result>();

  float urine = goal->urine_volume;
  float purified_total = 0.0f;
  int cycles = 0;

  while (urine > 0.0f && rclcpp::ok()) {
    float input = std::min(urine, 5.0f);
    urine -= input;
    ++cycles;

    float after_upa = input * 0.95f;
    float upa_temp = 40.0f + static_cast<float>(rand() % 30);
    if (upa_temp > upa_max_temperature_) {
      publish_diagnostics("UPA", true, "UPA temperature exceeded safe limit.");
      fail_goal(goal_handle, result, cycles, purified_total, "UPA overheating");
      return;
    }

    float after_filter = after_upa * 0.90f;
    float filter_temp = 40.0f + static_cast<float>(rand() % 20);
    if (filter_temp > filter_max_temperature_) {
      publish_diagnostics("Filter", true, "Filter temperature exceeded safe limit.");
      fail_goal(goal_handle, result, cycles, purified_total, "Filter overheating");
      return;
    }

    float after_ionization = after_filter * 0.98f;
    float ion_temp = 45.0f + static_cast<float>(rand() % 25);
    if (ion_temp > ionization_max_temperature_) {
      publish_diagnostics("Ionization", true, "Ionization chamber temperature exceeded safe limit.");
      fail_goal(goal_handle, result, cycles, purified_total, "Ionization overheating");
      return;
    }

    if (enable_failure_ && product_water_reserve_ + after_ionization > product_water_capacity_) {
      std::string msg = "Tank capacity exceeded: " + std::to_string(product_water_reserve_ + after_ionization) + " L";
      RCLCPP_FATAL(this->get_logger(), "%s", msg.c_str());
      publish_diagnostics("ProductWaterTank", true, msg);
      fail_goal(goal_handle, result, cycles, purified_total, "Tank full");
      return;
    }


    product_water_reserve_ += after_ionization;
    RCLCPP_INFO(this->get_logger(), "Purified %.2f L, Current reserve: %.2f L",
                after_ionization, product_water_reserve_);
    purified_total += after_ionization;

    feedback->time_step = cycles;
    feedback->current_tank_level = product_water_reserve_;
    feedback->current_purification_efficiency = after_ionization / input;
    feedback->failure_detected = false;
    feedback->unit_name = "WRS";

    
    goal_handle->publish_feedback(feedback);

    std::this_thread::sleep_for(500ms);

    RCLCPP_INFO(this->get_logger(), "Cycle %d: Processed %.2f L, Current reserve: %.2f L",
                cycles, after_ionization, product_water_reserve_);
  }

  result->success = true;
  result->summary_message = "WRS processing complete";
  result->total_purified_water = purified_total;
  result->total_cycles = cycles;

  RCLCPP_INFO(this->get_logger(),
      "[WRS RESULT] Success: %s, Total purified water: %.2f L, Cycles: %d, Msg: %s",
      result->success ? "true" : "false",
      result->total_purified_water,
      result->total_cycles,
      result->summary_message.c_str()
  );

  goal_handle->succeed(result);
  publish_diagnostics("WRS", false, result->summary_message);

  // After successful WRS execution, send water to OGS
  if (ogs_client_->wait_for_action_server(2s)) {
    space_station_eclss::action::OxygenGeneration::Goal ogs_goal;
    ogs_goal.input_water_mass = purified_total * 0.9f;

    auto send_goal_options = rclcpp_action::Client<space_station_eclss::action::OxygenGeneration>::SendGoalOptions();
    send_goal_options.result_callback = [this](const GoalHandleOGS::WrappedResult & result) {
      RCLCPP_INFO(this->get_logger(), "OGS result: O₂ generated = %.2f g, CH₄ vented = %.2f g",
                  result.result->total_o2_generated,
                  result.result->total_ch4_vented);
    };

    ogs_client_->async_send_goal(ogs_goal, send_goal_options);
  } else {
    RCLCPP_WARN(this->get_logger(), "OGS action server not available, skipping goal send.");
  }
}
void WRSActionServer::send_water_to_ogs(float volume, float iodine_ppm)
{
  if (!ogs_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(this->get_logger(), "OGS action server not available.");
    return;
  }

  space_station_eclss::action::OxygenGeneration::Goal goal;
  goal.input_water_mass = volume;

  auto send_goal_options = rclcpp_action::Client<space_station_eclss::action::OxygenGeneration>::SendGoalOptions();
  send_goal_options.result_callback = [this](const GoalHandleOGS::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "OGS processed water successfully.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "OGS processing failed");
    }
  };

  ogs_client_->async_send_goal(goal, send_goal_options);
}
void WRSActionServer::fail_goal(
  const std::shared_ptr<GoalHandleWRS> & goal_handle,
  std::shared_ptr<WRS::Result> & result,
  int cycles,
  float total,
  const std::string & reason)
{
  result->success = false;
  result->summary_message = "Failure: " + reason;
  result->total_purified_water = total;
  result->total_cycles = cycles;
  goal_handle->succeed(result);
  publish_diagnostics("WRS", true, reason);
}

void WRSActionServer::handle_product_water_request(
  const std::shared_ptr<space_station_eclss::srv::RequestProductWater::Request> request,
  std::shared_ptr<space_station_eclss::srv::RequestProductWater::Response> response)
{
  if (request->amount <= product_water_reserve_) {
    product_water_reserve_ -= request->amount;

    // Check if we fall below minimum threshold
    if (product_water_reserve_ < min_product_water_capacity_) {
      RCLCPP_FATAL(this->get_logger(), "Product water tank below minimum threshold: %.2f L", product_water_reserve_);
      publish_diagnostics("ProductWaterTank", true, "Tank below minimum safe capacity.");
    }
    
    response->water_granted = request->amount;
    response->success = true;
    response->message = "Water delivered";
  } else {
    response->water_granted = 0.0f;
    response->success = false;
    response->message = "Not enough reserve";
  }
  publish_reserve();
}


void WRSActionServer::handle_gray_water_request(
  const std::shared_ptr<space_station_eclss::srv::GreyWater::Request> request,
  std::shared_ptr<space_station_eclss::srv::GreyWater::Response> response)
{
  float volume = request->gray_water_liters;
  if (volume <= 0.0f) {
    response->success = false;
    response->message = "Invalid gray water input volume.";
    publish_diagnostics("WasteCollector", true, response->message);
    return;
  }

  if (waste_collector_current_ + volume > waste_collector_capacity_) {
    response->success = false;
    response->message = "Gray water rejected: Waste collector full";
    publish_diagnostics("WasteCollector", true, response->message);
  } else {
    waste_collector_current_ += volume;
    response->success = true;
    response->message = "Gray water accepted: " + std::to_string(volume) + " L";
    publish_diagnostics("WasteCollector", false, response->message);
  }
}

void WRSActionServer::publish_diagnostics(const std::string & unit, bool failure, const std::string & message)
{
  diagnostic_msgs::msg::DiagnosticStatus diag;
  diag.name = unit;
  diag.level = failure ? diagnostic_msgs::msg::DiagnosticStatus::ERROR : diagnostic_msgs::msg::DiagnosticStatus::OK;
  diag.message = message;
  diag.hardware_id = "WRS";
  diag_pub_->publish(diag);
}

void WRSActionServer::publish_reserve()
{
  std_msgs::msg::Float64 msg;
  msg.data = product_water_reserve_;
  reserve_pub_->publish(msg);
}

}  // namespace space_station_eclss

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<space_station_eclss::WRSActionServer>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
