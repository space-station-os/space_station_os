#include "space_station_eclss/ogs_system.hpp"

using namespace space_station_eclss;
using namespace std::chrono_literals;

OGSActionServer::OGSActionServer(const rclcpp::NodeOptions & options)
: Node("ogs_action_server", options)
{
  declare_parameters();

  action_server_ = rclcpp_action::create_server<OxygenGeneration>(
    this, "oxygen_generation",
    std::bind(&OGSActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&OGSActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&OGSActionServer::handle_accepted, this, std::placeholders::_1)
  );

  co2_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/co2_storage", 10,
    std::bind(&OGSActionServer::co2_callback, this, std::placeholders::_1));

  o2_pub_ = this->create_publisher<std_msgs::msg::Float64>("/o2_storage", 10);
  methane_pub_ = this->create_publisher<std_msgs::msg::Float64>("/methane_vented", 10);
  heartbeat_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/ogs/heartbeat", 10);

  grey_water_client_ = this->create_client<std_srvs::srv::Trigger>("/grey_water");
  co2_request_client_ = this->create_client<space_station_eclss::srv::CoRequest>("/ars/request_co2");

  o2_service_server_ = this->create_service<space_station_eclss::srv::O2Request>(
    "/ogs/request_o2",
    [this](const std::shared_ptr<space_station_eclss::srv::O2Request::Request> request,
           std::shared_ptr<space_station_eclss::srv::O2Request::Response> response) {
      if (latest_o2_ >= request->o2_req) {
        latest_o2_ -= request->o2_req;
        response->o2_resp = request->o2_req;
        response->success = true;
        response->message = "O2 request successful.";

        std_msgs::msg::Float64 o2_msg;
        o2_msg.data = latest_o2_;
        o2_pub_->publish(o2_msg);

      } else {
        response->o2_resp = 0.0;
        response->success = false;
        response->message = "Insufficient O2 available.";
      }
    });

  standby_timer_ = this->create_wall_timer(5s, std::bind(&OGSActionServer::standby_publish, this));
  RCLCPP_INFO(this->get_logger(), "OGS Action Server Initialized");
}

void OGSActionServer::declare_parameters()
{
  this->declare_parameter("enable_failure", true);
  this->declare_parameter("electrolysis_temp", 100.0);
  this->declare_parameter("o2_efficiency", 0.95);
  this->declare_parameter("sabatier_efficiency", 0.75);
  this->declare_parameter("sabatier_temp", 300.0);
  this->declare_parameter("sabatier_pressure", 1.0);

  this->get_parameter("enable_failure", enable_failure_);
  this->get_parameter("electrolysis_temp", electrolysis_temp_);
  this->get_parameter("o2_efficiency", o2_efficiency_);
  this->get_parameter("sabatier_efficiency", sabatier_efficiency_);
  this->get_parameter("sabatier_temp", sabatier_temp_);
  this->get_parameter("sabatier_pressure", sabatier_pressure_);
}

void OGSActionServer::co2_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  current_co2_ = msg->data;
}

rclcpp_action::GoalResponse OGSActionServer::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const OxygenGeneration::Goal> goal)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OGSActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleOGS> goal_handle)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void OGSActionServer::handle_accepted(const std::shared_ptr<GoalHandleOGS> goal_handle)
{
  std::thread{std::bind(&OGSActionServer::execute, this, goal_handle)}.detach();
}

void OGSActionServer::execute(const std::shared_ptr<GoalHandleOGS> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<OxygenGeneration::Feedback>();
  auto result = std::make_shared<OxygenGeneration::Result>();

  float iodine_removed = simulate_deionization(goal->iodine_concentration, goal->input_water_mass);
  if (iodine_removed < goal->iodine_concentration * 0.5f) {
    publish_heartbeat("Deionization failure", 2);
    if (enable_failure_) {
      result->success = false;
      result->summary_message = "Deionization failure";
      goal_handle->abort(result);
      return;
    }
  }

  auto [h2_generated, o2_generated] = simulate_electrolysis(goal->input_water_mass);

  // Request CO2 from ARS
  float co2_used = 0.0;
  if (co2_request_client_->wait_for_service(2s)) {
    auto request = std::make_shared<space_station_eclss::srv::CoRequest::Request>();
    request->co2_req = h2_generated * 4.0f;

    auto future = co2_request_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 1s) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      co2_used = future.get()->co2_resp;
    }
  }

  auto [ch4_vented, water_generated] = simulate_sabatier(h2_generated, co2_used);

  // Safety Checks
  float pio2_kpa = o2_generated * 101.3f / 21.0f;
  if (!check_environmental_limits(pio2_kpa, 80.0f, 10.0f)) {
    result->success = false;
    result->summary_message = "O2 environmental constraints violated";
    goal_handle->abort(result);
    return;
  }

  feedback->iodine_removed = iodine_removed;
  feedback->h2_generated = h2_generated;
  feedback->o2_generated = o2_generated;
  feedback->ch4_vented = ch4_vented;
  feedback->current_temperature = sabatier_temp_;
  goal_handle->publish_feedback(feedback);

  latest_o2_ = o2_generated;
  latest_ch4_ = ch4_vented;

  std_msgs::msg::Float64 o2_msg, ch4_msg;
  o2_msg.data = latest_o2_;
  ch4_msg.data = latest_ch4_;
  o2_pub_->publish(o2_msg);
  methane_pub_->publish(ch4_msg);

  if (grey_water_client_->wait_for_service(1s)) {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    grey_water_client_->async_send_request(req);
  }

  result->success = true;
  result->total_o2_generated = o2_generated;
  result->total_ch4_vented = ch4_vented;
  result->summary_message = "OGS operation complete";
  goal_handle->succeed(result);
}

float OGSActionServer::simulate_deionization(float iodine_ppm, float water_mass)
{
  return std::min(iodine_ppm, 2.0f * water_mass);
}

std::pair<float, float> OGSActionServer::simulate_electrolysis(float water_mass)
{
  float h2 = 0.111f * water_mass;
  float o2 = 0.889f * water_mass * o2_efficiency_;
  return {h2, o2};
}

std::pair<float, float> OGSActionServer::simulate_sabatier(float h2_mass, float co2_mass)
{
  float ch4 = std::min(h2_mass, co2_mass * 0.25f) * sabatier_efficiency_;
  float water = ch4 * 2.25f;
  return {ch4, water};
}

bool OGSActionServer::check_environmental_limits(float pio2, float pressure, float rate)
{
  if (pio2 < 19.5f || pio2 > 23.1f || pressure < 26.2f || pressure > 103.0f || rate > 13.5f) {
    publish_heartbeat("Environmental constraint violation", 2);
    return false;
  }
  return true;
}

void OGSActionServer::publish_heartbeat(const std::string &msg, int level)
{
  diagnostic_msgs::msg::DiagnosticStatus hb;
  hb.name = "OGS Subsystem Heartbeat";
  hb.hardware_id = "OGS_CONTROLLER";
  hb.message = msg;
  hb.level = level;
  heartbeat_pub_->publish(hb);
}

void OGSActionServer::standby_publish()
{
  publish_heartbeat("OGS subsystem in standby", 0);
  std_msgs::msg::Float64 o2_msg, ch4_msg;
  o2_msg.data = latest_o2_;
  ch4_msg.data = latest_ch4_;
  o2_pub_->publish(o2_msg);
  methane_pub_->publish(ch4_msg);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_options = rclcpp::NodeOptions();
  auto ogs_node = std::make_shared<OGSActionServer>(node_options);
  rclcpp::spin(ogs_node);
  rclcpp::shutdown();
  return 0;
}