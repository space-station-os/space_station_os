#include "space_station_eclss/wrs_systems.hpp"

#include <chrono>
#include <thread>
#include <cstdlib>
#include <ctime>

#include <ament_index_cpp/get_package_share_directory.hpp>


using namespace std::chrono_literals;

namespace space_station_eclss
{

WRSActionServer::WRSActionServer(const rclcpp::NodeOptions & options)
: Node("wrs_action_server", options),
  product_water_reserve_(1760.0),
  waste_collector_current_(0.0)
{
  
  std::srand(std::time(nullptr));  // seed RNG once

  // Declare with defaults
  this->declare_parameter<bool>("enable_failure", true);
  this->declare_parameter<double>("product_max_capacity", 2000.0);
  this->declare_parameter<double>("waste_max_capacity", 2000.0);
  this->declare_parameter<double>("product_min_capacity", 300.0);
  this->declare_parameter<double>("iodine_content", 2.0);  // mg/L
  this->declare_parameter<double>("upa_valve_pressure", 100.0);
  this->declare_parameter<double>("ionization_valve_pressure", 90.0);
  this->declare_parameter<double>("filter_valve_pressure", 85.0);
  this->declare_parameter<double>("catalytic_valve_pressure", 95.0);
  this->declare_parameter<double>("upa_max_temperature", 70.0);
  this->declare_parameter<double>("ionization_max_temperature", 90.0);
  this->declare_parameter<double>("filter_max_temperature", 60.0);
  this->declare_parameter<double>("catalytic_max_temperature", 80.0);

  // Then get values
  enable_failure_             = this->get_parameter("enable_failure").as_bool();
  product_water_capacity_     = this->get_parameter("product_max_capacity").as_double();
  waste_collector_capacity_   = this->get_parameter("waste_max_capacity").as_double();
  min_product_water_capacity_ = this->get_parameter("product_min_capacity").as_double();
  iodine_addition_            = this->get_parameter("iodine_content").as_double();
  upa_valve_pressure_         = this->get_parameter("upa_valve_pressure").as_double();
  ionization_valve_pressure_  = this->get_parameter("ionization_valve_pressure").as_double();
  filter_valve_pressure_      = this->get_parameter("filter_valve_pressure").as_double();
  catalytic_valve_pressure_   = this->get_parameter("catalytic_valve_pressure").as_double();
  upa_max_temperature_        = this->get_parameter("upa_max_temperature").as_double();
  ionization_max_temperature_ = this->get_parameter("ionization_max_temperature").as_double();
  filter_max_temperature_     = this->get_parameter("filter_max_temperature").as_double();
  catalytic_max_temperature_  = this->get_parameter("catalytic_max_temperature").as_double();

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
  auto result = std::make_shared<WRS::Result>();
  auto feedback = std::make_shared<WRS::Feedback>();

  float urine = goal->urine_volume;
  float purified_total = 0.0f;
  int cycles = 0;
  float stage_output = 0.0f;

  BT::BehaviorTreeFactory factory;

  // --- UPA stage (urine to distillate + brine) ---
  factory.registerSimpleAction("UPAStage", [&](BT::TreeNode &) {
    if (urine <= 0.0f) return BT::NodeStatus::SUCCESS;

    float input = std::min(urine, 5.0f); // process 5 L per cycle
    urine -= input;
    ++cycles;

    float distillate = input * 0.85f;   // 85% recovery
    float brine      = input - distillate; // 15% loss
    stage_output     = distillate;

    waste_collector_current_ += brine;  // brine sent to collector

    feedback->time_step = cycles;
    feedback->current_tank_level = product_water_reserve_;
    feedback->current_purification_efficiency = distillate / input;
    feedback->unit_name = "UPA";
    feedback->failure_detected = false;
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(this->get_logger(),
                "UPA Cycle %d: Input %.2f L, Distillate %.2f L, Brine %.2f L",
                cycles, input, distillate, brine);

    return BT::NodeStatus::SUCCESS;
  });

  // --- Filter stage (95% pass) ---
  factory.registerSimpleAction("FilterStage", [&](BT::TreeNode &) {
    float after_filter = stage_output * 0.95f;
    stage_output = after_filter;
    return BT::NodeStatus::SUCCESS;
  });

  // --- Ionization stage (98% pass) ---
  factory.registerSimpleAction("IonizationStage", [&](BT::TreeNode &) {
    float after_ion = stage_output * 0.98f;
    stage_output = after_ion;
    return BT::NodeStatus::SUCCESS;
  });

  // --- Tank stage ---
  factory.registerSimpleAction("TankStage", [&](BT::TreeNode &) {
    if (product_water_reserve_ + stage_output > product_water_capacity_) {
      std::string msg = "Tank capacity exceeded";
      publish_diagnostics("Tank", true, msg);
      fail_goal(goal_handle, result, cycles, purified_total, msg);
      return BT::NodeStatus::FAILURE;
    }
    product_water_reserve_ += stage_output;
    purified_total += stage_output;
    RCLCPP_INFO(this->get_logger(),
                "Cycle %d: Added %.2f L, Reserve now: %.2f",
                cycles, stage_output, product_water_reserve_);
    return BT::NodeStatus::SUCCESS;
  });

  // --- Grey Water Collection (service-driven only) ---
  factory.registerSimpleAction("CollectGreyWater", [&](BT::TreeNode &) {
    if (waste_collector_current_ <= 0.0f) {
      RCLCPP_INFO(this->get_logger(), "No grey water collected this cycle.");
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(this->get_logger(),
                "Grey water collector currently: %.2f L",
                waste_collector_current_);
    return BT::NodeStatus::SUCCESS;
  });

  // --- Grey Water Recycling (BPA, 80% recovery) ---
  factory.registerSimpleAction("RecycleGreyWater", [&](BT::TreeNode &) {
    if (waste_collector_current_ <= 0.0f) {
      RCLCPP_INFO(this->get_logger(), "No grey water to recycle.");
      return BT::NodeStatus::SUCCESS;
    }

    float recycled = waste_collector_current_ * 0.80f; // 80% recovery
    float residual = waste_collector_current_ - recycled;

    product_water_reserve_ += recycled;
    waste_collector_current_ = residual; // leave concentrated brine

    RCLCPP_INFO(this->get_logger(),
                "BPA recycled %.2f L grey water, %.2f L residual brine, Reserve now: %.2f",
                recycled, residual, product_water_reserve_);

    return BT::NodeStatus::SUCCESS;
  });

  std::string bt_xml_file = ament_index_cpp::get_package_share_directory("space_station_eclss") +
                            "/behaviortrees/wrs_bt.xml";
  auto tree = factory.createTreeFromFile(bt_xml_file);

  while (urine > 0.0f && rclcpp::ok()) {
    BT::NodeStatus status = tree.tickRoot();
    if (status == BT::NodeStatus::FAILURE) {
      return; // fail_goal already called
    }
    std::this_thread::sleep_for(500ms);
  }

  result->success = true;
  result->summary_message = "WRS BT processing complete";
  result->total_purified_water = purified_total;
  result->total_cycles = cycles;
  goal_handle->succeed(result);

  publish_diagnostics("WRS", false, result->summary_message);

  RCLCPP_INFO(this->get_logger(),"Purified Water : %.2f",purified_total);
  send_water_to_ogs(purified_total * 0.9f, iodine_addition_);
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
  goal_handle->abort(result);
  publish_diagnostics("WRS", true, reason);
}

void WRSActionServer::handle_product_water_request(
  const std::shared_ptr<space_station_eclss::srv::RequestProductWater::Request> request,
  std::shared_ptr<space_station_eclss::srv::RequestProductWater::Response> response)
{
  if (request->amount <= product_water_reserve_) {
    product_water_reserve_ -= request->amount;
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
