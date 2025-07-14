#include "space_station_eclss/ogs_system.hpp"

OGSSystem::OGSSystem() : Node("ogs_system")
{
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

  double water_granted = 0.0, co2_granted = 0.0;
  std::string water_msg, co2_msg;

  // === Step 1: Request Product Water ===
  if (!request_product_water(goal->input_water_mass, water_granted, water_msg)) {
    result->success = false;
    result->summary_message = "Water request failed: " + water_msg;
    goal_handle->abort(result);
    return;
  }

  // === Step 2: Electrolysis ===
  double o2_generated = 0.888 * water_granted;
  double h2_generated = 0.112 * water_granted;
  latest_o2_ += o2_generated;

  std_msgs::msg::Float64 o2_msg;
  o2_msg.data = latest_o2_;
  o2_pub_->publish(o2_msg);

  // === Step 3: Request CO2 from ARS ===
  if (!request_co2(co2_granted = 1.0 * h2_generated, co2_granted, co2_msg)) {
    result->success = false;
    result->summary_message = "CO2 request failed: " + co2_msg;
    goal_handle->abort(result);
    return;
  }

  // === Step 4: Sabatier Reactor ===
  double ch4 = 0.5 * co2_granted;
  total_ch4_vented_ += ch4;

  std_msgs::msg::Float64 ch4_msg;
  ch4_msg.data = total_ch4_vented_;
  ch4_pub_->publish(ch4_msg);

  double gray_water = 0.8 * water_granted;  // assume 80% recovery
  std::string gray_msg;
  if (!send_gray_water(gray_water, gray_msg)) {
    result->success = false;
    result->summary_message = "Gray water transfer failed: " + gray_msg;
    goal_handle->abort(result);
    return;
  }

  feedback->time_step = 1;
  feedback->current_temperature = 293.15;
  feedback->o2_generated = o2_generated;
  feedback->h2_generated = h2_generated;
  feedback->ch4_vented = ch4;
  goal_handle->publish_feedback(feedback);

  publish_diagnostics("O2 Generation Successful");

  result->success = true;
  result->total_o2_generated = o2_generated;
  result->total_ch4_vented = ch4;
  result->summary_message = "O2 and CH4 generation complete.";
  goal_handle->succeed(result);


}

bool OGSSystem::request_product_water(double amount, double &granted, std::string &msg)
{
  if (!water_client_->wait_for_service(std::chrono::seconds(2))) {
    msg = "Product water service unavailable";
    return false;
  }

  auto req = std::make_shared<space_station_eclss::srv::RequestProductWater::Request>();
  req->amount = amount;

  auto future = water_client_->async_send_request(req);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
    msg = "Failed to call product water service";
    return false;
  }

  auto resp = future.get();
  granted = resp->water_granted;
  msg = resp->message;
  return resp->success;
}

bool OGSSystem::request_co2(double co2_mass_kg, double &granted_ppco2, std::string &msg)
{
  if (!co2_client_->wait_for_service(std::chrono::seconds(2))) {
    msg = "CO₂ service unavailable";
    return false;
  }

  
  const double R = 8.314;           // J/(mol·K)
  const double T = 295.0;           // Cabin temperature in Kelvin
  const double V = 100.0;           // Cabin volume in m³
  const double MOLAR_MASS_CO2 = 44.01; // g/mol

 
  double co2_mass_g = co2_mass_kg * 1000.0;
  double n_moles = co2_mass_g / MOLAR_MASS_CO2;
  double pressure_pa = (n_moles * R * T) / V;
  double pressure_mmHg = pressure_pa * 760.0 / 101325.0;

 
  auto req = std::make_shared<space_station_eclss::srv::Co2Request::Request>();
  req->co2_req = pressure_mmHg;

  // Send request
  auto future = co2_client_->async_send_request(req);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
    msg = "Failed to call CO₂ service";
    return false;
  }

  auto resp = future.get();
  granted_ppco2 = resp->co2_resp;
  msg = resp->message;
  return resp->success;
}
bool OGSSystem::send_gray_water(double amount_liters, std::string &msg)
{
  if (!gray_water_client_->wait_for_service(std::chrono::seconds(2))) {
    msg = "Gray water service unavailable.";
    return false;
  }

  auto req = std::make_shared<space_station_eclss::srv::GreyWater::Request>();
  req->gray_water_liters = amount_liters;

  auto future = gray_water_client_->async_send_request(req);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
    msg = "Failed to call gray water service.";
    return false;
  }

  auto resp = future.get();
  msg = resp->message;
  return resp->success;
}

void OGSSystem::o2_service_callback(
  const std::shared_ptr<space_station_eclss::srv::O2Request::Request> request,
  std::shared_ptr<space_station_eclss::srv::O2Request::Response> response)
{
  double amount = request->o2_req;

  if (latest_o2_ >= amount) {
    latest_o2_ -= amount;
    response->o2_resp = amount;
    response->success = true;
    response->message = "O2 request successful.";

    // Publish updated O2 level
    std_msgs::msg::Float64 o2_msg;
    o2_msg.data = latest_o2_;
    o2_pub_->publish(o2_msg);
  } else {
    response->o2_resp = 0.0;
    response->success = false;
    response->message = "Insufficient O2 available.";
  }
}

void OGSSystem::publish_diagnostics(const std::string &status)
{
  diagnostic_msgs::msg::DiagnosticStatus diag;
  diag.name = "OGS Status";
  diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  diag.message = status;
  diag_pub_->publish(diag);
}
int main (int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto ogs_system = std::make_shared<OGSSystem>();
  rclcpp::spin(ogs_system);
  rclcpp::shutdown();
  return 0;
}
