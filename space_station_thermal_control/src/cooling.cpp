#include "space_station_thermal_control/cooling.hpp"

using namespace std::chrono_literals;

namespace space_station_thermal_control
{
CoolantActionServer::CoolantActionServer(const rclcpp::NodeOptions &options)
: Node("coolant_heat_transfer_action_server", options), goal_counter_(0), lasted_vented_heat_(0.0)
{
  this->declare_parameter("mass_kg", 200.0);
  this->declare_parameter("specific_heat_j_per_kg_c", 4186.0);
  this->declare_parameter("heat_transfer_efficiency", 0.85);
  this->declare_parameter("vent_threshold_kj", 250.0);
  this->declare_parameter("diagnostics_enabled", true);

  this->get_parameter("mass_kg", mass_kg_);
  this->get_parameter("specific_heat_j_per_kg_c", cp_water_);
  this->get_parameter("heat_transfer_efficiency", transfer_efficiency_);
  this->get_parameter("vent_threshold_kj", vent_threshold_kj_);
  this->get_parameter("diagnostics_enabled", diagnostics_enabled_);

  internal_loop_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>(
      "/tcs/internal_loop_heat", 10);
  external_loop_pub_ = this->create_publisher<space_station_thermal_control::msg::ExternalLoopStatus>(
      "/tcs/external_loop_a/status", 10);

  if (diagnostics_enabled_)
  {
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
        "/thermals/diagnostics", 10);
  }

  product_water_client_ = this->create_client<space_station_eclss::srv::RequestProductWater>(
      "/wrs/product_water_request");

  grey_water_client_ = this->create_client<std_srvs::srv::Trigger>(
      "/grey_water");

  venting_server_ = this->create_service<space_station_thermal_control::srv::VentHeat>(
      "/tcs/radiator_a/vent_heat",
      std::bind(&CoolantActionServer::handleVenting, this, std::placeholders::_1, std::placeholders::_2));

  action_server_ = rclcpp_action::create_server<Coolant>(
      shared_from_this(),
      "coolant_heat_transfer",
      std::bind(&CoolantActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CoolantActionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&CoolantActionServer::handleAccepted, this, std::placeholders::_1));

  current_temp_c_ = 25.0;
  timer_ = this->create_wall_timer(5s, std::bind(&CoolantActionServer::simulateHeatRise, this));

  RCLCPP_INFO(this->get_logger(),
              "Coolant action server ready. Mass: %.1f kg, Specific Heat: %.1f J/kgC, Efficiency: %.2f, Vent Threshold: %.1f kJ",
              mass_kg_, cp_water_, transfer_efficiency_, vent_threshold_kj_);
}

void CoolantActionServer::simulateHeatRise()
{
  current_temp_c_ += 1.0;
  RCLCPP_INFO(this->get_logger(), "[SIM] Current internal temperature: %.2f C", current_temp_c_);
}

rclcpp_action::GoalResponse CoolantActionServer::handleGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Coolant::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "[ACTION] Cooling goal received for %s with input temp %.2f C",
              goal->component_id.c_str(), goal->input_temperature_c);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CoolantActionServer::handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_WARN(this->get_logger(), "[ACTION] Cancel request received.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CoolantActionServer::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&CoolantActionServer::execute, this, goal_handle)}.detach();
}

void CoolantActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<Coolant::Result>();

  double delta_T = current_temp_c_ - goal->input_temperature_c;
  if (delta_T <= 0.0)
  {
    RCLCPP_WARN(this->get_logger(), "[ACTION] No cooling needed. Current: %.2f C, Goal: %.2f C",
                current_temp_c_, goal->input_temperature_c);
    result->success = true;
    result->message = "No cooling needed";
    goal_handle->succeed(result);
    return;
  }

  double Q_joules = mass_kg_ * cp_water_ * delta_T;
  double Q_kj = Q_joules / 1000.0;

  RCLCPP_INFO(this->get_logger(),
              "[ACTION] Cooling %s from %.2fC to %.2fC (%.2f kJ heat removed)",
              goal->component_id.c_str(), current_temp_c_, goal->input_temperature_c, Q_kj);

  current_temp_c_ = goal->input_temperature_c;
  publishInternalLoop();

  double ammonia_heat = Q_kj * transfer_efficiency_;
  publishExternalLoop(ammonia_heat);

  if (ammonia_heat >= vent_threshold_kj_)
  {
    std::this_thread::sleep_for(500ms);
    RCLCPP_WARN(this->get_logger(),
                "[ACTION] Venting triggered (%.2f kJ exceeds %.2f kJ)",
                ammonia_heat, vent_threshold_kj_);
    lasted_vented_heat_ = ammonia_heat;
  }

  if (++goal_counter_ >= 10)
  {
    recycleWater();
    goal_counter_ = 0;
  }

  if (diagnostics_enabled_)
  {
    publishDiagnostics(true, "Cooling operation successful");
  }

  result->success = true;
  result->message = "Cooling completed successfully";
  goal_handle->succeed(result);
}

void CoolantActionServer::publishInternalLoop()
{
  sensor_msgs::msg::Temperature msg;
  msg.temperature = current_temp_c_;
  msg.variance = 0.1;
  internal_loop_pub_->publish(msg);
}

void CoolantActionServer::publishExternalLoop(double received_heat_kj)
{
  space_station_thermal_control::msg::ExternalLoopStatus msg;
  msg.received_heat_kj = received_heat_kj;
  msg.loop_inlet_temp = 6.0;
  msg.loop_outlet_temp = 12.0;
  msg.ammonia_inlet_temp = 4.0;
  msg.ammonia_outlet_temp = 10.0;
  external_loop_pub_->publish(msg);
}

void CoolantActionServer::recycleWater()
{
  auto water_req = std::make_shared<space_station_eclss::srv::RequestProductWater::Request>();
  water_req->amount = 5.0;
  if (product_water_client_->wait_for_service(1s))
  {
    product_water_client_->async_send_request(water_req);
  }

  auto grey_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  if (grey_water_client_->wait_for_service(1s))
  {
    grey_water_client_->async_send_request(grey_req);
  }
}

void CoolantActionServer::handleVenting(
    const std::shared_ptr<space_station_thermal_control::srv::VentHeat::Request> req,
    std::shared_ptr<space_station_thermal_control::srv::VentHeat::Response> res)
{
  if (lasted_vented_heat_ >= req->excess_heat)
  {
    lasted_vented_heat_ -= req->excess_heat;
    res->success = true;
    res->message = "Venting successful";
  }
  else
  {
    res->success = false;
    res->message = "Insufficient ventable heat";
  }
}

void CoolantActionServer::publishDiagnostics(bool status, const std::string &msg)
{
  diagnostic_msgs::msg::DiagnosticStatus diag;
  diag.level = status ? diag.OK : diag.ERROR;
  diag.name = "CoolantServer";
  diag.message = msg;
  diag.hardware_id = "space_station_tcs_001";
  diag_pub_->publish(diag);
}

} // namespace space_station_thermal_control

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<space_station_thermal_control::CoolantActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
