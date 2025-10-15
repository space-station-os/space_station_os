#include "space_station_eclss/ars_systems.hpp"

#include <chrono>
#include <random>

using namespace std::chrono_literals;

namespace space_station_eclss
{

ARSActionServer::ARSActionServer(const rclcpp::NodeOptions & options)
: Node("ars_systems_node", options)
{
  declare_parameters();

  RCLCPP_INFO(this->get_logger(), "Switching on ARS — requesting power from DDCU...");
  load_client_ = this->create_client<space_station_eps::srv::Load>("/ddcu/load_request");

  powered_ = false;
  RCLCPP_INFO(this->get_logger(), "Waiting for EPS power...");

  // Retry every 2s until supply_load() returns true
  power_retry_timer_ = this->create_wall_timer(2s, [this]() {
    if (!powered_) {
      if (supply_load()) {
        powered_ = true;
        RCLCPP_INFO(this->get_logger(), "ARS powered successfully — initializing systems.");
        initialize_systems();
        power_retry_timer_->cancel();
      } else {
        RCLCPP_WARN(this->get_logger(), "Still waiting for DDCU power...");
      }
    }
  });
}

void ARSActionServer::declare_parameters()
{
  this->declare_parameter("sim_time", 10);
  this->declare_parameter("enable_failure", true);
  this->declare_parameter("max_co2_storage", 3947.4);
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


void ARSActionServer::initialize_systems()
{
  using GoalHandleAirRevitalisation = rclcpp_action::ServerGoalHandle<AirRevitalisation>;

  this->action_server_ = rclcpp_action::create_server<AirRevitalisation>(
    this,
    "air_revitalisation",
    std::bind(&ARSActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ARSActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ARSActionServer::execute, this, std::placeholders::_1));

  heartbeat_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/ars/diagnostics", 10);

  co2_storage_pub_ = this->create_publisher<std_msgs::msg::Float64>("/co2_storage", 10);
  co2_pub_timer_ = this->create_wall_timer(2s, [this]() {
    std_msgs::msg::Float64 msg;
    msg.data = total_co2_storage_;
    co2_storage_pub_->publish(msg);
  });

  co2_request_srv_ = this->create_service<Co2Request>(
    "/ars/request_co2",
    std::bind(&ARSActionServer::handle_co2_service, this, std::placeholders::_1, std::placeholders::_2)
  );

  combustion_timer_ = this->create_wall_timer(10s, [this]() { monitor_combustion_and_contaminants(); });

  contaminant_timer_ = this->create_wall_timer(3s, [this]() {
    contaminant_level_ += 1.0f;
    contaminant_level_ = std::min(contaminant_level_, 500.0f);
    if (contaminant_level_ > contaminant_limit_) {
      publish_bed_heartbeat("ARS", false, "Contaminant threshold exceeded", "Contaminant_Monitor");
    }
  });

  disable_failure_ = this->create_subscription<std_msgs::msg::Bool>(
    "/ars/self_diagnosis",
    10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      this->set_parameter(rclcpp::Parameter("enable_failure", msg->data));
      enable_failure_ = msg->data;
      RCLCPP_INFO(this->get_logger(), "Failure simulation %s", enable_failure_ ? "enabled" : "disabled");
    }
  );


  
}

void ARSActionServer::execute(const std::shared_ptr<GoalHandleARS> goal_handle)
{

  if (!powered_) {
    RCLCPP_WARN(this->get_logger(), "ARS cannot start — system not powered.");
    auto result = std::make_shared<AirRevitalisation::Result>();
    result->success = false;
    result->summary_message = "ARS unpowered — unable to start process.";
    goal_handle->abort(result);
    return;
  }

  auto feedback = std::make_shared<AirRevitalisation::Feedback>();
  auto result = std::make_shared<AirRevitalisation::Result>();

  float co2 = goal_handle->get_goal()->initial_co2_mass;
  float h2o = goal_handle->get_goal()->initial_moisture_content;
  float contaminants = goal_handle->get_goal()->initial_contaminants;
  float temp1 = 30.0, temp2 = 30.0, temp3 = 30.0, temp4 = 30.0;

  int vents = 0;

  BT::BehaviorTreeFactory factory;

  // Desiccants
  factory.registerSimpleAction("Desiccant1", [&](BT::TreeNode &) {
    return simulate_desiccant_bed(h2o, des1_capacity_, des1_removal_, des1_temp_limit_, "Desiccant1", temp1)
             ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });

  factory.registerSimpleAction("Desiccant2", [&](BT::TreeNode &) {
    return simulate_desiccant_bed(h2o, des2_capacity_, des2_removal_, des2_temp_limit_, "Desiccant2", temp2)
             ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });

  // Adsorbents
  factory.registerSimpleAction("Adsorbent1", [&](BT::TreeNode &) {
    return simulate_adsorbent_bed(co2, ads1_capacity_, ads1_removal_, ads1_temp_limit_, "Adsorbent1", temp3)
             ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });

  factory.registerSimpleAction("Adsorbent2", [&](BT::TreeNode &) {
    return simulate_adsorbent_bed(co2, ads2_capacity_, ads2_removal_, ads2_temp_limit_, "Adsorbent2", temp4)
             ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });

  // Vent / Store CO₂
  factory.registerSimpleAction("VentOrStoreCO2", [&](BT::TreeNode &) {
    double vented = co2 * 0.5;
    double stored = co2 - vented;
    total_co2_storage_ += stored;

    feedback->cabin_co2_level = co2;
    feedback->venting = true;
    feedback->vent_bed_id = 1;
    feedback->vent_amount = vented;
    goal_handle->publish_feedback(feedback);

    vents++;
    return BT::NodeStatus::SUCCESS;
  });

 std::string bt_xml_file = ament_index_cpp::get_package_share_directory("space_station_eclss") +
                          "/behaviortrees/ars_bt.xml";

  // RCLCPP_INFO(this->get_logger(), "Loading BT from: %s", bt_xml_file.c_str());

  auto tree = factory.createTreeFromFile(bt_xml_file);

  
  for (int t = 0; t < sim_time_; ++t) {
    if (!rclcpp::ok() || goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }

    BT::NodeStatus status = tree.tickRoot();

    if (status == BT::NodeStatus::FAILURE) {
      result->success = false;
      result->summary_message = "ARS BT failed (bed failure).";
      goal_handle->abort(result);
      return;
    }

    contaminant_level_ = std::max(0.0f, contaminant_level_ - 2.0f);
    rclcpp::sleep_for(100ms);
  }

  if (total_co2_storage_ > max_co2_storage_) {
    double excess = total_co2_storage_ - max_co2_storage_;
    total_co2_storage_ = max_co2_storage_;  

    RCLCPP_WARN(this->get_logger(),
                "CO₂ storage exceeded safe limit by %.2f g. Venting excess...",
                excess);

    // Publish venting diagnostic
    publish_bed_heartbeat("ARS", true,
                          "Auto-venting triggered due to CO₂ buildup",
                          "CO2_storage");

    feedback->venting = true;
    feedback->vent_bed_id = 99;  // special "safety vent"
    feedback->vent_amount = excess;
    goal_handle->publish_feedback(feedback);

    vents++;
  }


  std_msgs::msg::Float64 co2_msg;
  co2_msg.data = total_co2_storage_;
  co2_storage_pub_->publish(co2_msg);

  result->success = true;
  result->cycles_completed = sim_time_;
  result->total_vents = vents;
  result->total_co2_vented = co2 / 2.0;
  result->summary_message = "ARS process completed with fallback BT sequence";

  goal_handle->succeed(result);
  
}

// ------------------- SUPPORT METHODS -------------------

rclcpp_action::GoalResponse ARSActionServer::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const AirRevitalisation::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with CO₂: %.2f", goal->initial_co2_mass);

  
  if (total_co2_storage_ > max_co2_storage_) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal: CO₂ storage already above max threshold (%.2f > %.2f)",
                total_co2_storage_, max_co2_storage_);
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ARSActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleARS>)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

bool ARSActionServer::simulate_desiccant_bed(float &h2o, float cap, float rate, float max_temp, const std::string &name, float &temp)
{
  if (h2o <= 0) return true;
  h2o -= rate;
  temp += 10.0;
  if (temp > max_temp) {
    publish_bed_heartbeat("ARS", false, "Overheating", name);
    return false;
  }
  return true;
}

bool ARSActionServer::simulate_adsorbent_bed(float &co2, float cap, float rate, float max_temp, const std::string &name, float &temp)
{
  if (co2 <= 0) return true;
  co2 -= rate;
  temp += 12.0;
  if (temp > max_temp) {
    publish_bed_heartbeat("ARS", false, "Overheating", name);
    return false;
  }
  return true;
}

void ARSActionServer::publish_bed_heartbeat(const std::string &bed, bool ok, const std::string &msg, const std::string &hardware_id)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = bed;
  status.level = ok ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.message = msg;
  status.hardware_id = hardware_id;
  heartbeat_pub_->publish(status);
}

void ARSActionServer::monitor_combustion_and_contaminants()
{
  if (enable_failure_ && (std::rand() % 1000 < 3)) {
    publish_bed_heartbeat("ARS", false, "Possible toxic combustion products detected", "ARS_Monitor");
  }
  if (enable_failure_ && (std::rand() % 1000 < 5)) {
    publish_bed_heartbeat("ARS", false, "Atmospheric anomaly detected", "ARS_Monitor");
  }
}

bool ARSActionServer::supply_load()
{
  if (!load_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "DDCU load service not available yet.");
    return false;
  }

  auto request = std::make_shared<space_station_eps::srv::Load::Request>();
  request->load_voltage = 124.5;

  load_client_->async_send_request(request,
    [this](rclcpp::Client<space_station_eps::srv::Load>::SharedFuture future_resp) {
      auto response = future_resp.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Power granted: %s", response->message.c_str());
        powered_ = true;   // initialization done by timer on next tick
      } else {
        RCLCPP_ERROR(this->get_logger(), "DDCU rejected load: %s", response->message.c_str());
        powered_ = false;
      }
    });

  return true;  // request sent
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
    // Reduce from storage
    total_co2_storage_ -= req->co2_req;

    res->co2_resp = req->co2_req;
    res->success = true;
    res->message = "CO2 successfully delivered";

    RCLCPP_INFO(this->get_logger(),
                "CO₂ request of %.2f accepted. Remaining storage: %.2f",
                req->co2_req, total_co2_storage_);
  } else {
    res->co2_resp = 0.0;
    res->success = false;
    res->message = "Insufficient CO₂ in storage";

    RCLCPP_WARN(this->get_logger(),
                "CO₂ request of %.2f rejected. Only %.2f left in storage.",
                req->co2_req, total_co2_storage_);
  }

  // Publish updated CO₂ storage every time
  std_msgs::msg::Float64 msg;
  msg.data = total_co2_storage_;
  co2_storage_pub_->publish(msg);
}

}  // namespace space_station_eclss

