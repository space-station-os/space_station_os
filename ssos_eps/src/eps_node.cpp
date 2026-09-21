#include "ssos_eps/eps_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>

namespace ssos_eps
{

using std::placeholders::_1;

EpsNode::EpsNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("eps_node", options)
{
  this->declare_parameter("step_rate_hz", step_rate_hz_);

  this->declare_parameter(
    "solar_array_area_m2", solar_array_area_m2_);

  this->declare_parameter(
    "solar_array_efficiency", solar_array_efficiency_);

  this->declare_parameter(
    "load_demand_w", load_demand_w_);

  this->declare_parameter(
    "battery_capacity_wh", battery_capacity_wh_);

  this->declare_parameter(
    "initial_battery_soc", initial_battery_soc_);

  this->declare_parameter(
    "low_soc_threshold", low_soc_threshold_);

  this->declare_parameter(
    "enable_auto_faults", enable_auto_faults_);
}

CallbackReturn EpsNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  step_rate_hz_ =
    this->get_parameter("step_rate_hz").as_double();

  solar_array_area_m2_ =
    this->get_parameter("solar_array_area_m2").as_double();

  solar_array_efficiency_ =
    this->get_parameter("solar_array_efficiency").as_double();

  load_demand_w_ =
    this->get_parameter("load_demand_w").as_double();

  battery_capacity_wh_ =
    this->get_parameter("battery_capacity_wh").as_double();

  initial_battery_soc_ =
    this->get_parameter("initial_battery_soc").as_double();

  low_soc_threshold_ =
    this->get_parameter("low_soc_threshold").as_double();

  enable_auto_faults_ =
    this->get_parameter("enable_auto_faults").as_bool();

  EpsConfig config;
  config.solar_array_area_m2 = solar_array_area_m2_;
  config.solar_array_efficiency = solar_array_efficiency_;
  config.load_demand_w = load_demand_w_;
  config.battery_capacity_wh = battery_capacity_wh_;
  config.initial_battery_soc = initial_battery_soc_;
  config.low_soc_threshold = low_soc_threshold_;

  model_ = std::make_unique<EpsModel>(config);
  last_state_ = model_->state();

  telemetry_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/ssos/eps/diagnostics", 10);

  heartbeat_pub_ =
    this->create_publisher<SubsystemHeartbeat>(
    "/ssos/eps/heartbeat", 10);

  fault_pub_ =
    this->create_publisher<FaultEvent>(
    "/ssos/fault_event", 10);

  world_state_sub_ =
    this->create_subscription<WorldState>(
    "/sim/world_state",
    10,
    std::bind(&EpsNode::on_world_state, this, _1));

  register_client_ =
    this->create_client<RegisterSubsystem>(
    "/ssos/register_subsystem");

  low_soc_fault_published_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn EpsNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  telemetry_pub_->on_activate();
  heartbeat_pub_->on_activate();
  fault_pub_->on_activate();

  first_step_ = true;

  const auto period =
    std::chrono::duration<double>(
    1.0 / std::max(step_rate_hz_, 1.0e-3));

  step_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&EpsNode::step, this));

  register_with_manager();

  return CallbackReturn::SUCCESS;
}

CallbackReturn EpsNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (step_timer_) {
    step_timer_->cancel();
    step_timer_.reset();
  }

  telemetry_pub_->on_deactivate();
  heartbeat_pub_->on_deactivate();
  fault_pub_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn EpsNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  step_timer_.reset();

  telemetry_pub_.reset();
  heartbeat_pub_.reset();
  fault_pub_.reset();

  world_state_sub_.reset();
  register_client_.reset();

  model_.reset();

  low_soc_fault_published_ = false;

  return CallbackReturn::SUCCESS;
}

void EpsNode::on_world_state(
  const WorldState::SharedPtr msg)
{
  solar_flux_w_m2_ = msg->solar_flux_w_m2;
  in_eclipse_ = msg->in_eclipse;
}

void EpsNode::step()
{
  if (!model_) {
    return;
  }

  const rclcpp::Time now = this->now();

  // Advance energy state only when ROS time advances.
  // When simulation time is paused or has not started, dt remains zero.
  double dt = 0.0;

  if (!first_step_) {
    const double measured =
      (now - last_step_time_).seconds();

    if (measured > 0.0) {
      dt = measured;
    }
  }

  first_step_ = false;
  last_step_time_ = now;

  EpsInput input;
  input.solar_flux_w_m2 = solar_flux_w_m2_;
  input.in_eclipse = in_eclipse_;
  input.dt_s = dt;

  last_state_ = model_->update(input);

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = now;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "eps";
  status.hardware_id = "ssos_eps/eps";

  auto add_value =
    [&status](const std::string & key, double value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = std::to_string(value);
      status.values.push_back(pair);
    };

  add_value(
    "generation_w",
    last_state_.generation_w);

  add_value(
    "load_demand_w",
    last_state_.load_demand_w);

  add_value(
    "battery_soc",
    last_state_.battery_soc);

  add_value(
    "power_balance_w",
    last_state_.power_balance_w);

  add_value(
    "solar_flux_w_m2",
    solar_flux_w_m2_);

  status.level =
    last_state_.healthy ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;

  status.message =
    last_state_.healthy ?
    "nominal" :
    "battery state of charge below threshold";

  diagnostics.status.push_back(status);
  telemetry_pub_->publish(diagnostics);

  SubsystemHeartbeat heartbeat;
  heartbeat.stamp = now;
  heartbeat.subsystem_name = "eps";
  heartbeat.lifecycle_state =
    SubsystemHeartbeat::LIFECYCLE_ACTIVE;

  heartbeat.healthy = last_state_.healthy;
  heartbeat.status_message = status.message;

  heartbeat_pub_->publish(heartbeat);

  if (
    enable_auto_faults_ &&
    !last_state_.healthy &&
    !low_soc_fault_published_)
  {
    FaultEvent fault;

    fault.stamp = now;
    fault.subsystem_name = "eps";
    fault.fault_type = "low_battery_soc";
    fault.severity = FaultEvent::SEVERITY_CRITICAL;
    fault.description =
      "EPS battery state of charge is below the configured threshold";

    fault.affected_interfaces = {
      "/ssos/eps/diagnostics",
      "/ssos/eps/heartbeat"
    };

    fault_pub_->publish(fault);
    low_soc_fault_published_ = true;
  }
}

void EpsNode::register_with_manager()
{
  if (!register_client_->wait_for_service(
      std::chrono::milliseconds(200)))
  {
    RCLCPP_WARN(
      get_logger(),
      "system_manager unavailable; continuing");

    return;
  }

  auto request =
    std::make_shared<RegisterSubsystem::Request>();

  request->subsystem_name = "eps";

  request->published_topics = {
    "/ssos/eps/diagnostics"
  };

  request->subscribed_topics = {
    "/sim/world_state"
  };

  request->heartbeat_topic =
    "/ssos/eps/heartbeat";

  register_client_->async_send_request(request);
}

}  // namespace ssos_eps
