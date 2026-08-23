#include "ssos_thermal/nodes/coolant_node.hpp"

#include <chrono>
#include <future>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

namespace ssos_thermal
{
namespace nodes
{

CoolantNode::CoolantNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("coolant_node", options)
{
  this->declare_parameter("mass_kg", mass_kg_);
  this->declare_parameter("specific_heat_j_per_kg_c", specific_heat_j_per_kg_c_);
  this->declare_parameter("heat_transfer_efficiency", heat_transfer_efficiency_);
  this->declare_parameter("vent_threshold_kj", vent_threshold_kj_);
  this->declare_parameter("target_temp_c", target_temp_c_);

  autostart_timer_ = ThermalDiagnostics::maybe_autostart(this);
}

CallbackReturn CoolantNode::on_configure(const rclcpp_lifecycle::State &)
{
  mass_kg_ = this->get_parameter("mass_kg").as_double();
  specific_heat_j_per_kg_c_ = this->get_parameter("specific_heat_j_per_kg_c").as_double();
  heat_transfer_efficiency_ = this->get_parameter("heat_transfer_efficiency").as_double();
  vent_threshold_kj_ = this->get_parameter("vent_threshold_kj").as_double();
  target_temp_c_ = this->get_parameter("target_temp_c").as_double();

  loop_ = std::make_unique<coolant::CoolantLoop>(coolant::CoolantParams{
    mass_kg_, specific_heat_j_per_kg_c_, heat_transfer_efficiency_, vent_threshold_kj_});

  heartbeat_pub_ = this->create_publisher<SubsystemHeartbeat>("/ssos/coolant/heartbeat", 10);
  fault_pub_ = this->create_publisher<FaultEvent>("/ssos/fault_event", 10);

  radiator_client_ = this->create_client<space_station_interfaces::srv::VentHeat>(
    "/tcs/radiator_a/vent_heat");
  register_client_ = this->create_client<RegisterSubsystem>("/ssos/register_subsystem");

  action_server_ = rclcpp_action::create_server<Coolant>(
    this, "coolant_heat_transfer",
    std::bind(&CoolantNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&CoolantNode::handleCancel, this, std::placeholders::_1),
    std::bind(&CoolantNode::handleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
              "Coolant node configured. Mass: %.1f kg, cp: %.1f J/kgC, "
              "efficiency: %.2f, vent threshold: %.1f kJ",
              mass_kg_, specific_heat_j_per_kg_c_, heat_transfer_efficiency_,
              vent_threshold_kj_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn CoolantNode::on_activate(const rclcpp_lifecycle::State &)
{
  heartbeat_pub_->on_activate();
  fault_pub_->on_activate();

  heartbeat_timer_ = this->create_wall_timer(
    1s, std::bind(&CoolantNode::publishHeartbeat, this));

  registerWithManager();
  RCLCPP_INFO(get_logger(), "Coolant node activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CoolantNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (heartbeat_timer_) {
    heartbeat_timer_->cancel();
    heartbeat_timer_.reset();
  }
  heartbeat_pub_->on_deactivate();
  fault_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CoolantNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  heartbeat_timer_.reset();
  heartbeat_pub_.reset();
  fault_pub_.reset();
  action_server_.reset();
  radiator_client_.reset();
  register_client_.reset();
  loop_.reset();
  return CallbackReturn::SUCCESS;
}

void CoolantNode::publishHeartbeat()
{
  heartbeat_pub_->publish(ThermalDiagnostics::make_heartbeat(
    this->now(), "coolant", SubsystemHeartbeat::LIFECYCLE_ACTIVE, true, "nominal"));
}

void CoolantNode::registerWithManager()
{
  if (!register_client_->wait_for_service(std::chrono::milliseconds(200))) {
    RCLCPP_WARN(get_logger(),
                "system_manager registration service unavailable; continuing");
    return;
  }
  auto req = std::make_shared<RegisterSubsystem::Request>();
  req->subsystem_name = "coolant";
  req->published_topics = {};
  req->subscribed_topics = {};
  req->heartbeat_topic = "/ssos/coolant/heartbeat";
  register_client_->async_send_request(req);
}

rclcpp_action::GoalResponse CoolantNode::handleGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Coolant::Goal> goal)
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(get_logger(), "[ACTION] Rejecting goal: node not active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "[ACTION] Cooling goal received for %s with input temp %.2f C",
              goal->component_id.c_str(), goal->input_temperature_c);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CoolantNode::handleCancel(
  const std::shared_ptr<GoalHandleCoolant>)
{
  RCLCPP_WARN(get_logger(), "[ACTION] Cancel request received.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CoolantNode::handleAccepted(const std::shared_ptr<GoalHandleCoolant> goal_handle)
{
  std::thread{[this, goal_handle]() {this->execute(goal_handle);}}.detach();
}

void CoolantNode::execute(const std::shared_ptr<GoalHandleCoolant> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<Coolant::Result>();

  RCLCPP_INFO(get_logger(), "[ACTION] Cooling %s from %.2f C towards coolant %.2f C",
              goal->component_id.c_str(), goal->input_temperature_c, target_temp_c_);

  double node_temp = goal->input_temperature_c;
  double vented_total = 0.0;

  while (rclcpp::ok() && node_temp > target_temp_c_ + 0.5) {
    const coolant::CoolStepResult step = loop_->step(node_temp, target_temp_c_);
    node_temp = step.node_temp_c;

    if (step.vent_triggered) {
      vented_total += step.ammonia_heat_kj;
      RCLCPP_WARN(get_logger(), "[ACTION] Venting triggered (%.2f kJ exceeds %.2f kJ)",
                  step.ammonia_heat_kj, vent_threshold_kj_);
    }

    auto feedback = std::make_shared<Coolant::Feedback>();
    feedback->internal_temp_c = node_temp;
    feedback->ammonia_temp_c = step.ammonia_temp_c;
    feedback->vented_heat_kj = vented_total;
    goal_handle->publish_feedback(feedback);

    rclcpp::sleep_for(100ms);
  }

  result->success = true;
  result->message = "Cooling completed successfully";
  goal_handle->succeed(result);

  RCLCPP_INFO(get_logger(), "[ACTION] Cooling complete: final node temp = %.2f C, "
              "total vented = %.2f kJ", node_temp, vented_total);

  if (vented_total > 0.0) {
    auto req = std::make_shared<space_station_interfaces::srv::VentHeat::Request>();
    req->excess_heat = vented_total;

    if (radiator_client_->wait_for_service(2s)) {
      auto future = radiator_client_->async_send_request(req);
      if (future.wait_for(2s) == std::future_status::ready) {
        auto resp = future.get();
        if (resp->success) {
          RCLCPP_INFO(get_logger(), "[RADIATOR] %s", resp->message.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "[RADIATOR] Failed: %s", resp->message.c_str());
        }
      } else {
        RCLCPP_ERROR(get_logger(), "[RADIATOR] VentHeat service timed out");
      }
    } else {
      RCLCPP_ERROR(get_logger(), "[RADIATOR] VentHeat service not available");
    }
  }
}

}  // namespace nodes
}  // namespace ssos_thermal
