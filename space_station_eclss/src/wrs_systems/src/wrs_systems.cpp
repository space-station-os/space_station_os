#include "space_station_eclss/wrs_systems.hpp"

#include <chrono>
#include <thread>
#include <string>

using namespace std::chrono_literals;

namespace space_station_eclss
{

WRSActionServer::WRSActionServer(const rclcpp::NodeOptions & options)
: Node("wrs_action_server", options),
  product_water_reserve_(0.0),
  waste_collector_current_(0.0)
{
  // Declare parameters
  enable_failure_ = this->declare_parameter("enable_failure", true);

  product_water_capacity_ = this->declare_parameter("product_max_capacity", 100.0f);
  waste_collector_capacity_ = this->declare_parameter("waste_max_capacity", 50.0f);

  upa_valve_pressure_ = this->declare_parameter("upa_valve_pressure", 100.0f);
  ionization_valve_pressure_ = this->declare_parameter("ionization_valve_pressure", 90.0f);
  filter_valve_pressure_ = this->declare_parameter("filter_valve_pressure", 85.0f);
  catalytic_valve_pressure_ = this->declare_parameter("catalytic_valve_pressure", 95.0f);

  upa_max_temperature_ = this->declare_parameter("upa_max_temperature", 70.0f);
  ionization_max_temperature_ = this->declare_parameter("ionization_max_temperature", 65.0f);
  filter_max_temperature_ = this->declare_parameter("filter_max_temperature", 60.0f);
  catalytic_max_temperature_ = this->declare_parameter("catalytic_max_temperature", 80.0f);

  // Action server with 4 callbacks
  action_server_ = rclcpp_action::create_server<WRS>(
    this,
    "water_recovery_systems",
    std::bind(&WRSActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&WRSActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&WRSActionServer::handle_accepted, this, std::placeholders::_1)
  );

  // Product water request service
  water_request_server_ = this->create_service<space_station_eclss::srv::RequestProductWater>(
    "wrs/product_water_request",
    std::bind(&WRSActionServer::handle_product_water_request, this, std::placeholders::_1, std::placeholders::_2)
  );

  gray_water_service_ = this->create_service<space_station_eclss::srv::GreyWater>(
    "/grey_water",
    std::bind(&WRSActionServer::handle_gray_water_request, this,
              std::placeholders::_1, std::placeholders::_2)
  );


  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("wrs/diagnostics", 10);
  reserve_pub_ = this->create_publisher<std_msgs::msg::Float32>("wrs/product_water_reserve", 10);
  reserve_timer_ = this->create_wall_timer(2s, std::bind(&WRSActionServer::publish_reserve, this));

  RCLCPP_INFO(this->get_logger(), "WRS Action Server initialized");
}

rclcpp_action::GoalResponse WRSActionServer::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const WRS::Goal> goal)
{
  if (goal->urine_volume <= 0.0f) {
    RCLCPP_WARN(this->get_logger(), "Rejected goal: invalid urine volume");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WRSActionServer::handle_cancel(const std::shared_ptr<GoalHandleWRS> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Cancel request received.");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
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
    float input = std::min(urine, 5.0f);  // 5L per cycle
    urine -= input;
    ++cycles;

    float pressure_factor = (upa_valve_pressure_ + ionization_valve_pressure_ +
                             filter_valve_pressure_ + catalytic_valve_pressure_) / 400.0f;
    float purified = input * 0.85f * pressure_factor;

    if (enable_failure_ && product_water_reserve_ + purified > product_water_capacity_) {
      publish_diagnostics("ProductTank", true, "Overcapacity failure");
      result->success = false;
      result->summary_message = "Failure: product tank full";
      result->total_cycles = cycles;
      result->total_purified_water = purified_total;
      goal_handle->succeed(result);
      return;
    }

    product_water_reserve_ += purified;
    purified_total += purified;

    feedback->time_step = cycles;
    feedback->current_tank_level = product_water_reserve_;
    feedback->current_purification_efficiency = 0.85f * pressure_factor;
    feedback->failure_detected = false;
    feedback->unit_name = "WRS";
    goal_handle->publish_feedback(feedback);

    std::this_thread::sleep_for(500ms);
  }

  result->success = true;
  result->summary_message = "Processing complete";
  result->total_purified_water = purified_total;
  result->total_cycles = cycles;
  goal_handle->succeed(result);
  publish_diagnostics("WRS", false, "Processing complete");
}

void WRSActionServer::handle_product_water_request(
  const std::shared_ptr<space_station_eclss::srv::RequestProductWater::Request> request,
  std::shared_ptr<space_station_eclss::srv::RequestProductWater::Response> response)
{
  if (request->amount <= product_water_reserve_) {
    product_water_reserve_ -= request->amount;
    response->success = true;
    response->message = "Water delivered";
  } else {
    response->success = false;
    response->message = "Not enough reserve";
  }
  publish_reserve();
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
  std_msgs::msg::Float32 msg;
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
