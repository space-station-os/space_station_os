#include "ssos_core/system_manager.hpp"

#include <functional>
#include <chrono>

namespace ssos_core
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

SystemManager::SystemManager(const rclcpp::NodeOptions & options)
: LifecycleNode("system_manager", options)
{
  // Declare parameters
  this->declare_parameter("heartbeat_timeout_s", 5.0);
  this->declare_parameter("eval_period_s", 1.0);

  RCLCPP_INFO(get_logger(), "SystemManager created (unconfigured)");
}

CallbackReturn SystemManager::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring...");

  // Read parameters
  heartbeat_timeout_s_ = this->get_parameter("heartbeat_timeout_s").as_double();
  eval_period_s_ = this->get_parameter("eval_period_s").as_double();

  // Create state publisher (lifecycle-managed — only publishes when active)
  state_pub_ = this->create_publisher<SystemState>("/ssos/system_state", 10);

  // Subscribe to fault events
  fault_sub_ = this->create_subscription<FaultEvent>(
    "/ssos/fault_event", 10,
    std::bind(&SystemManager::on_fault_event, this, _1));

  // Registration service
  register_srv_ = this->create_service<RegisterSubsystem>(
    "/ssos/register_subsystem",
    std::bind(&SystemManager::handle_register, this, _1, _2));

  // Periodic state evaluation timer
  auto eval_period = std::chrono::duration<double>(eval_period_s_);
  eval_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(eval_period),
    std::bind(&SystemManager::run_evaluation, this));

  current_state_ = SystemState::INIT;

  RCLCPP_INFO(get_logger(), "Configured. Heartbeat timeout=%.1fs, eval period=%.1fs",
    heartbeat_timeout_s_, eval_period_s_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn SystemManager::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating...");
  state_pub_->on_activate();
  publish_state(SystemState::INIT, "system_manager activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SystemManager::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");
  state_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SystemManager::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up...");
  state_pub_.reset();
  fault_sub_.reset();
  register_srv_.reset();
  eval_timer_.reset();
  subsystems_.clear();
  current_state_ = SystemState::INIT;
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Service: subsystem registration
// ---------------------------------------------------------------------------
void SystemManager::handle_register(
  const RegisterSubsystem::Request::SharedPtr req,
  RegisterSubsystem::Response::SharedPtr res)
{
  const auto & name = req->subsystem_name;

  if (subsystems_.count(name) > 0) {
    RCLCPP_WARN(get_logger(), "Subsystem '%s' already registered, updating", name.c_str());
  }

  // Create record
  SubsystemRecord record;
  record.name = name;
  record.published_topics = req->published_topics;
  record.subscribed_topics = req->subscribed_topics;
  record.heartbeat_topic = req->heartbeat_topic;
  record.registered = true;
  record.last_heard = this->now();

  // Initialize heartbeat with defaults
  record.last_heartbeat.subsystem_name = name;
  record.last_heartbeat.lifecycle_state = SubsystemHeartbeat::LIFECYCLE_UNCONFIGURED;
  record.last_heartbeat.healthy = false;
  record.last_heartbeat.status_message = "awaiting first heartbeat";

  // Create dynamic subscription for this subsystem's heartbeat
  record.heartbeat_sub = this->create_subscription<SubsystemHeartbeat>(
    req->heartbeat_topic, 10,
    [this, name](const SubsystemHeartbeat::SharedPtr msg) {
      this->on_heartbeat(name, msg);
    });

  subsystems_[name] = std::move(record);

  res->accepted = true;
  res->message = "Registered subsystem '" + name + "', listening on " + req->heartbeat_topic;

  RCLCPP_INFO(get_logger(), "Registered subsystem '%s' (heartbeat: %s)",
    name.c_str(), req->heartbeat_topic.c_str());

  // Re-evaluate immediately
  run_evaluation();
}

// ---------------------------------------------------------------------------
// Subscription callbacks
// ---------------------------------------------------------------------------
void SystemManager::on_heartbeat(
  const std::string & subsystem_name,
  const SubsystemHeartbeat::SharedPtr msg)
{
  auto it = subsystems_.find(subsystem_name);
  if (it == subsystems_.end()) {
    RCLCPP_WARN(get_logger(), "Heartbeat from unregistered subsystem '%s'",
      subsystem_name.c_str());
    return;
  }

  it->second.last_heartbeat = *msg;
  it->second.last_heard = this->now();

  // Immediate re-evaluation on health change
  run_evaluation();
}

void SystemManager::on_fault_event(const FaultEvent::SharedPtr msg)
{
  RCLCPP_WARN(get_logger(), "Fault event: subsystem=%s type=%s severity=%d desc='%s'",
    msg->subsystem_name.c_str(),
    msg->fault_type.c_str(),
    msg->severity,
    msg->description.c_str());

  // Emergency faults go straight to SAFE
  if (msg->severity == FaultEvent::SEVERITY_EMERGENCY) {
    publish_state(SystemState::SAFE,
      "EMERGENCY fault from " + msg->subsystem_name + ": " + msg->description);
    return;
  }

  // For non-emergency faults, let the normal evaluation handle it
  run_evaluation();
}

// ---------------------------------------------------------------------------
// State evaluation
// ---------------------------------------------------------------------------
uint8_t SystemManager::evaluate_system_state() const
{
  // If in SAFE mode, stay there (requires manual intervention)
  if (current_state_ == SystemState::SAFE) {
    return SystemState::SAFE;
  }

  // No subsystems registered yet — still booting
  if (subsystems_.empty()) {
    return SystemState::INIT;
  }

  bool all_active = true;
  bool all_healthy = true;

  auto now = this->now();

  for (const auto & [name, record] : subsystems_) {
    // Check for heartbeat timeout
    double elapsed = (now - record.last_heard).seconds();
    bool timed_out = elapsed > heartbeat_timeout_s_;

    bool is_active =
      record.last_heartbeat.lifecycle_state == SubsystemHeartbeat::LIFECYCLE_ACTIVE;

    if (!is_active || timed_out) {
      all_active = false;
    }

    if (!record.last_heartbeat.healthy || timed_out) {
      all_healthy = false;
    }
  }

  if (all_active && all_healthy) {
    return SystemState::NOMINAL;
  }

  return SystemState::DEGRADED;
}

void SystemManager::run_evaluation()
{
  uint8_t new_state = evaluate_system_state();

  if (new_state != current_state_) {
    // Build reason string
    std::string reason;
    switch (new_state) {
      case SystemState::NOMINAL:
        reason = "all subsystems active and healthy";
        break;
      case SystemState::DEGRADED: {
        reason = "degraded subsystems:";
        auto now = this->now();
        for (const auto & [name, record] : subsystems_) {
          double elapsed = (now - record.last_heard).seconds();
          bool timed_out = elapsed > heartbeat_timeout_s_;
          if (!record.last_heartbeat.healthy ||
            record.last_heartbeat.lifecycle_state != SubsystemHeartbeat::LIFECYCLE_ACTIVE ||
            timed_out)
          {
            reason += " " + name;
            if (timed_out) {
              reason += "(timeout)";
            }
          }
        }
        break;
      }
      case SystemState::INIT:
        reason = "no subsystems registered";
        break;
      default:
        reason = "state transition";
        break;
    }
    publish_state(new_state, reason);
  }
}

void SystemManager::publish_state(uint8_t new_state, const std::string & reason)
{
  const char * state_names[] = {"INIT", "NOMINAL", "DEGRADED", "RECOVERY", "SAFE"};
  const char * old_name = (current_state_ <= 4) ? state_names[current_state_] : "UNKNOWN";
  const char * new_name = (new_state <= 4) ? state_names[new_state] : "UNKNOWN";

  RCLCPP_INFO(get_logger(), "State transition: %s -> %s (%s)",
    old_name, new_name, reason.c_str());

  current_state_ = new_state;

  auto msg = SystemState();
  msg.stamp = this->now();
  msg.state = new_state;
  msg.triggering_event = reason;

  // Build degraded subsystems list
  auto now = this->now();
  for (const auto & [name, record] : subsystems_) {
    double elapsed = (now - record.last_heard).seconds();
    bool timed_out = elapsed > heartbeat_timeout_s_;
    if (!record.last_heartbeat.healthy ||
      record.last_heartbeat.lifecycle_state != SubsystemHeartbeat::LIFECYCLE_ACTIVE ||
      timed_out)
    {
      msg.degraded_subsystems.push_back(name);
    }
  }
  state_pub_->publish(msg);
}
}  // namespace ssos_core