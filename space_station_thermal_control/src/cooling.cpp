#include "space_station_thermal_control/cooling.hpp"

using namespace std::chrono_literals;

namespace space_station_thermal_control
{
CoolantActionServer::CoolantActionServer(const rclcpp::NodeOptions &options)
: Node("coolant_server", options), goal_counter_(0), lasted_vented_heat_(0.0)
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
      this,
      "coolant_heat_transfer",
      std::bind(&CoolantActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CoolantActionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&CoolantActionServer::handleAccepted, this, std::placeholders::_1));

  coolant_temp_c_ = 25.0;
  // timer_ = this->create_wall_timer(5s, std::bind(&CoolantActionServer::simulateHeatRise, this));

  RCLCPP_INFO(this->get_logger(),
              "Coolant action server ready. Mass: %.1f kg, Specific Heat: %.1f J/kgC, Efficiency: %.2f, Vent Threshold: %.1f kJ",
              mass_kg_, cp_water_, transfer_efficiency_, vent_threshold_kj_);
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
  std::thread{[this, goal_handle]() {
    this->execute(goal_handle);
  }}.detach(); 
}


void CoolantActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<Coolant::Result>();

  RCLCPP_INFO(this->get_logger(),
              "[ACTION] Cooling %s from %.2f C towards coolant %.2f C",
              goal->component_id.c_str(), goal->input_temperature_c, coolant_temp_c_);

  double target_temp = coolant_temp_c_;
  double node_temp   = goal->input_temperature_c;


  double vented_total = 0.0;

  while (rclcpp::ok() && node_temp > target_temp + 0.5) {
    // Simulate cooling step
    double delta_T = std::min(2.5, node_temp - target_temp);
    double Q_joules = mass_kg_ * cp_water_ * delta_T;
    double Q_kj = Q_joules / 1000.0;

    node_temp -= delta_T;

    // Heat transferred to ammonia
    double ammonia_heat = Q_kj * transfer_efficiency_;
    double ammonia_temp = 5.0 + (ammonia_heat / 1000.0); // dummy model

    // Vent if needed
    if (ammonia_heat >= vent_threshold_kj_) {
      vented_total += ammonia_heat;
      RCLCPP_WARN(this->get_logger(),
                  "[ACTION] Venting triggered (%.2f kJ exceeds %.2f kJ)",
                  ammonia_heat, vent_threshold_kj_);
    }

    // Publish feedback
    auto feedback = std::make_shared<Coolant::Feedback>();
    feedback->internal_temp_c = node_temp;
    feedback->ammonia_temp_c  = ammonia_temp;
    feedback->vented_heat_kj  = vented_total;
    goal_handle->publish_feedback(feedback);

    // Sleep for loop rate
    rclcpp::sleep_for(100ms);

  }

  // Final result
  result->success = true;
  result->message = "Cooling completed successfully";
  goal_handle->succeed(result);

  RCLCPP_INFO(this->get_logger(),
              "[ACTION] Cooling complete: final node temp = %.2f C, total vented = %.2f kJ",
              node_temp, vented_total);
}


void CoolantActionServer::publishInternalLoop()
{
  sensor_msgs::msg::Temperature msg;
  msg.temperature = coolant_temp_c_;
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
