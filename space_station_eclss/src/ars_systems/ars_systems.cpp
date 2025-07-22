#include "space_station_eclss/ars_systems.hpp"

#include <chrono>
#include <random>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace space_station_eclss {

ARSActionServer::ARSActionServer(const rclcpp::NodeOptions & options)
: Node("ars_systems_node", options)
{
  declare_parameters();

  using GoalHandleAirRevitalisation = rclcpp_action::ServerGoalHandle<AirRevitalisation>;

  this->action_server_ = rclcpp_action::create_server<AirRevitalisation>(
    this,
    "air_revitalisation",
    std::bind(&ARSActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ARSActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ARSActionServer::execute, this, std::placeholders::_1));


  heartbeat_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/ars/heartbeat", 10);
  heartbeat_timer_ = this->create_wall_timer(1s, [this]() {
    publish_bed_heartbeat("ARS Monitor", true, "System nominal");
  });
  co2_storage_pub_ = this->create_publisher<std_msgs::msg::Float64>("/co2_storage", 10);
  co2_pub_timer_ = this->create_wall_timer(2s, [this]() {
    std_msgs::msg::Float64 msg;
    msg.data = total_co2_storage_;
    co2_storage_pub_->publish(msg);
  });

  co2_request_srv_ = this->create_service<Co2Request>(
    "/ars/request_co2",
    std::bind(&ARSActionServer::handle_co2_service, this, _1, _2)
  );

  combustion_timer_ = this->create_wall_timer(10s, [this]() { monitor_combustion_and_contaminants(); });

  contaminant_timer_ = this->create_wall_timer(3s, [this]() {
    contaminant_level_ += static_cast<float>(std::rand() % 4);
    contaminant_level_ = std::min(contaminant_level_, 500.0f);  // arbitrary cap
    if (contaminant_level_ > contaminant_limit_) {
      publish_bed_heartbeat("Atmosphere", false, "Contaminant threshold exceeded");
    } else {
      publish_bed_heartbeat("Atmosphere", true, "Contaminant levels within safe range");
    }
  });

 disable_failure_ = this->create_subscription<std_msgs::msg::Bool>(
    "/ars/self_diagnosis",
    10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      enable_failure_ = msg->data;
      RCLCPP_INFO(this->get_logger(), "Failure simulation %s", enable_failure_ ? "enabled" : "disabled");
    }
  );


  RCLCPP_INFO(this->get_logger(), "ARS action server ready.");
}

void ARSActionServer::declare_parameters()
{
  this->declare_parameter("sim_time", 10);
  this->declare_parameter("enable_failure", true);
  this->declare_parameter("max_co2_storage", 3947.0);  // 3 mmHg ≈ 3947 ppm
  this->declare_parameter("contaminant_limit", 100.0);

  this->declare_parameter("des1_capacity", 100.0);
  this->declare_parameter("des1_removal", 1.5);
  this->declare_parameter("des1_temp_limit", 120.0);

  this->declare_parameter("des2_capacity", 100.0);
  this->declare_parameter("des2_removal", 1.5);
  this->declare_parameter("des2_temp_limit", 120.0);

  this->declare_parameter("ads1_capacity", 100.0);
  this->declare_parameter("ads1_removal", 2.5);
  this->declare_parameter("ads1_temp_limit", 204.0);

  this->declare_parameter("ads2_capacity", 100.0);
  this->declare_parameter("ads2_removal", 2.5);
  this->declare_parameter("ads2_temp_limit", 204.0);

  sim_time_ = this->get_parameter("sim_time").as_int();
  enable_failure_ = this->get_parameter("enable_failure").as_bool();
  max_co2_storage_ = this->get_parameter("max_co2_storage").as_double();
  contaminant_limit_ = this->get_parameter("contaminant_limit").as_double();

  des1_capacity_ = this->get_parameter("des1_capacity").as_double();
  des1_removal_ = this->get_parameter("des1_removal").as_double();
  des1_temp_limit_ = this->get_parameter("des1_temp_limit").as_double();

  des2_capacity_ = this->get_parameter("des2_capacity").as_double();
  des2_removal_ = this->get_parameter("des2_removal").as_double();
  des2_temp_limit_ = this->get_parameter("des2_temp_limit").as_double();

  ads1_capacity_ = this->get_parameter("ads1_capacity").as_double();
  ads1_removal_ = this->get_parameter("ads1_removal").as_double();
  ads1_temp_limit_ = this->get_parameter("ads1_temp_limit").as_double();

  ads2_capacity_ = this->get_parameter("ads2_capacity").as_double();
  ads2_removal_ = this->get_parameter("ads2_removal").as_double();
  ads2_temp_limit_ = this->get_parameter("ads2_temp_limit").as_double();
}

void ARSActionServer::execute(const std::shared_ptr<GoalHandleARS> goal_handle)
{
  auto feedback = std::make_shared<AirRevitalisation::Feedback>();
  auto result = std::make_shared<AirRevitalisation::Result>();

  float co2 = goal_handle->get_goal()->initial_co2_mass;
  float h2o = goal_handle->get_goal()->initial_moisture_content;
  float contaminants = goal_handle->get_goal()->initial_contaminants;
  float temp1 = 30.0, temp2 = 30.0, temp3 = 230.0, temp4 = 230.0;

  int vents = 0;

  for (int t = 0; t < sim_time_; ++t) {
    if (!rclcpp::ok()) {
      goal_handle->canceled(result);
      return;
    }

    bool d1 = simulate_desiccant_bed(h2o, des1_capacity_, des1_removal_, des1_temp_limit_, "Desiccant_1", temp1);
    bool d2 = simulate_desiccant_bed(h2o, des2_capacity_, des2_removal_, des2_temp_limit_, "Desiccant_2", temp2);
    bool a1 = simulate_adsorbent_bed(co2, ads1_capacity_, ads1_removal_, ads1_temp_limit_, "Adsorbent_1", temp3);
    bool a2 = simulate_adsorbent_bed(co2, ads2_capacity_, ads2_removal_, ads2_temp_limit_, "Adsorbent_2", temp4);

    if (!(d1 && d2 && a1 && a2)) {
      result->success = false;
      result->summary_message = "Bed failure during ARS simulation.";
      goal_handle->abort(result);
      return;
    }

    feedback->current_time = t;
    feedback->cabin_co2_level = co2;
    feedback->venting = true;
    feedback->vent_bed_id = 1;
    feedback->vent_amount = co2;
    goal_handle->publish_feedback(feedback);
    vents++;

    contaminant_level_ = std::max(0.0f, contaminant_level_ - 2.0f);
    rclcpp::sleep_for(100ms);
  }

  total_co2_storage_ += co2;
  if (total_co2_storage_ > max_co2_storage_) {
    publish_bed_heartbeat("CO2_Storage", false, "CO2 partial pressure exceeds 3mmHg");
    result->success = false;
    result->summary_message = "Exceeded CO2 storage pressure limit";
    goal_handle->abort(result);
    return;
  }

  std_msgs::msg::Float64 co2_msg;
  co2_msg.data = total_co2_storage_;
  co2_storage_pub_->publish(co2_msg);

  result->success = true;
  result->cycles_completed = sim_time_;
  result->total_vents = vents;
  result->total_co2_vented = co2;
  result->summary_message = "ARS process completed";
  goal_handle->succeed(result);
}
rclcpp_action::GoalResponse ARSActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const AirRevitalisation::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with CO₂: %.2f", goal->initial_co2_mass);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ARSActionServer::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<AirRevitalisation>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
// CDRA BED
bool ARSActionServer::simulate_desiccant_bed(float &h2o, float cap, float rate, float max_temp, const std::string &name, float &temp)
{
  if (h2o <= 0) return true;
  h2o -= rate;
  temp += 10.0;
  if (temp > max_temp) {
    publish_bed_heartbeat(name, false, "Overheating");
    return false;
  }
  publish_bed_heartbeat(name, true, "Nominal");
  return true;
}

bool ARSActionServer::simulate_adsorbent_bed(float &co2, float cap, float rate, float max_temp, const std::string &name, float &temp)
{
  if (co2 <= 0) return true;
  co2 -= rate;
  temp += 12.0;
  if (temp > max_temp) {
    publish_bed_heartbeat(name, false, "Overheating");
    return false;
  }
  publish_bed_heartbeat(name, true, "Nominal");
  return true;
}

void ARSActionServer::publish_bed_heartbeat(const std::string &bed, bool ok, const std::string &msg)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = bed;
  status.level = ok ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.message = msg;
  heartbeat_pub_->publish(status);
}

void ARSActionServer::monitor_combustion_and_contaminants()
{
  if (enable_failure_ && (std::rand() % 1000 < 3)) {
    publish_bed_heartbeat("Combustion Detector", false, "Possible toxic combustion products detected");
  }
  if (enable_failure_ && (std::rand() % 1000 < 5)) {
    publish_bed_heartbeat("Unknown Contaminant", false, "Atmospheric anomaly detected");
  }
}

void ARSActionServer::handle_co2_service(
  const std::shared_ptr<Co2Request::Request> req,
  std::shared_ptr<Co2Request::Response> res)
{
  if (req->co2_req <= 0.0) {
    res->co2_resp = 0.0;
    res->success = false;
    res->message = "Invalid request amount";
    return;
  }

  if (total_co2_storage_ >= req->co2_req) {
    total_co2_storage_ -= req->co2_req;
    res->co2_resp = req->co2_req;
    res->success = true;
    res->message = "CO2 successfully delivered";
  } else {
    res->co2_resp = 0.0;
    res->success = false;
    res->message = "Insufficient CO2 in storage";
  }

  std_msgs::msg::Float64 msg;
  msg.data = total_co2_storage_;
  co2_storage_pub_->publish(msg);
}

}  // namespace space_station_eclss



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node_options = rclcpp::NodeOptions();
  auto ars_node = std::make_shared<space_station_eclss::ARSActionServer>(node_options);

  rclcpp::spin(ars_node);

  rclcpp::shutdown();
  return 0;
}