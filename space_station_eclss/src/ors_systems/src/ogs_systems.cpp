#include "space_station_eclss/ogs_system.hpp"

namespace space_station_eclss {

OGSSystem::OGSSystem(const rclcpp::NodeOptions & options)
: Node("ogs_system", options)
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

  RCLCPP_INFO(this->get_logger(), "Switching on OGS — requesting power from DDCU...");
  load_client_ = this->create_client<space_station_eps::srv::Load>("/ddcu/load_request");

  powered_ = false;
  RCLCPP_INFO(this->get_logger(), "Waiting for EPS power...");

  // Retry every 2s until supply_load() returns true
  power_retry_timer_ = this->create_wall_timer(2s, [this]() {
    if (!powered_) {
      if (supply_load()) {
        powered_ = true;
        RCLCPP_INFO(this->get_logger(), "OGS powered successfully — initializing systems.");
        initialize_systems();
        power_retry_timer_->cancel();
      } else {
        RCLCPP_WARN(this->get_logger(), "Still waiting for DDCU power...");
      }
    }
  });
  
}

void OGSSystem::initialize_systems()
{
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

  co2_storage_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/co2_storage", rclcpp::QoS(10),
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      co2_storage_.store(msg->data, std::memory_order_relaxed);
    }
    );

  
  last_co2_request_time_ = this->now() - rclcpp::Duration::from_seconds(30.0);  
  RCLCPP_INFO(this->get_logger(), "OGS system ready.");
}



bool OGSSystem::supply_load()
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

void OGSSystem::execute_goal(const std::shared_ptr<GoalHandleOGS> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<OxygenGeneration::Result>();
  auto feedback = std::make_shared<OxygenGeneration::Feedback>();

  BT::BehaviorTreeFactory factory;

  // Register BT nodes inline
  factory.registerSimpleAction("RequestProductWater", [&](BT::TreeNode &) {
    request_product_water(5.0);
    return BT::NodeStatus::SUCCESS;
  });

  factory.registerSimpleAction("Electrolysis", [&](BT::TreeNode &) {
    double water_litres = goal->input_water_mass;     // litres of H2O
    double water_mass_g = water_litres * 1000.0;      // convert L → g (1 L = 1000 g)

    // Mass split: 88.9% O2, 11.1% H2
    double o2_generated = 0.889 * water_mass_g;       // g O2
    double h2_generated = 0.111 * water_mass_g;       // g H2

    latest_o2_ += o2_generated;

    if (enable_failure_ && latest_o2_ > max_o2_capacity_) {
      publish_failure_diagnostics("O2Storage", "Overflow during Electrolysis");
      return BT::NodeStatus::FAILURE;
    }

    feedback->o2_generated = o2_generated;
    feedback->h2_generated = h2_generated;
    goal_handle->publish_feedback(feedback);

    return BT::NodeStatus::SUCCESS;
  });


  factory.registerSimpleAction("SabatierReactor", [&, this](BT::TreeNode &) {
    // Throttle to 30 s
    const rclcpp::Time now = this->now();
    const auto since_last = now - last_co2_request_time_;
    const bool time_to_request = since_last >= rclcpp::Duration::from_seconds(30.0);

    // Latest CO₂ storage value (from subscription)
    const double co2_storage = co2_storage_.load(std::memory_order_relaxed);

    // Only request if enough CO₂ is available
    if (time_to_request && co2_storage > 1e-6) {   // >0 g available
      const double co2_req = 0.30 * co2_storage;
      request_co2(co2_req);
      last_co2_request_time_ = now;
    }

    // H₂ feedback → generate CH₄ + H₂O
    double h2_generated = feedback->h2_generated;

    double ch4_generated = 2.0 * h2_generated;   // g CH₄
    double grey_water    = 4.5 * h2_generated;   // g H₂O

    total_ch4_vented_ += ch4_generated;

    std_msgs::msg::Float64 ch4_msg;
    ch4_msg.data = ch4_generated;
    ch4_pub_->publish(ch4_msg);

    return BT::NodeStatus::SUCCESS;
});

  factory.registerSimpleAction("SendGreyWater", [&](BT::TreeNode &) {
    send_gray_water(0.9 * goal->input_water_mass);
    return BT::NodeStatus::SUCCESS;
  });

  // Load XML
  bt_xml_file_ = ament_index_cpp::get_package_share_directory("space_station_eclss") + "/behaviortrees/ogs_bt.xml";
  auto tree = factory.createTreeFromFile(bt_xml_file_);

  // RCLCPP_INFO(this->get_logger(), "Executing OGS BT from: %s", bt_xml_file_.c_str());

  BT::NodeStatus status = tree.tickRoot();
  if (status == BT::NodeStatus::FAILURE) {
    result->success = false;
    result->summary_message = "OGS BT failed.";
    goal_handle->abort(result);
    return;
  }

  result->success = true;
  result->total_o2_generated = feedback->o2_generated;
  result->total_ch4_vented = total_ch4_vented_;
  result->summary_message = "OGS process completed.";
  goal_handle->succeed(result);
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
  
  ch4_msg.data = total_ch4_vented_;
  ch4_pub_->publish(ch4_msg);

  // Publish diagnostics
  diagnostic_msgs::msg::DiagnosticStatus diag;
  diag.name = "OGSSystem Heartbeat";
  diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  diag.message = "OGS running. Latest O2: " + std::to_string(latest_o2_) + " g";
  diag_pub_->publish(diag);
}


}