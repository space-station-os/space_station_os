#include "ssos_eclss/nodes/ogs_node.hpp"

#include <functional>
#include <vector>

#include "ssos_eclss/nodes/eclss_diagnostics.hpp"

using std::placeholders::_1;

namespace ssos_eclss
{
namespace nodes
{

OgsNode::OgsNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("ogs_node", options)
{
  this->declare_parameter("step_rate_hz", step_rate_hz_);
  this->declare_parameter("stack_current_a", stack_current_a_);
  this->declare_parameter("o2_required_kg_day", o2_required_kg_day_);
  this->declare_parameter("enable_auto_faults", enable_auto_faults_);
  autostart_timer_ = EclssDiagnostics::maybe_autostart(this);
}

CallbackReturn OgsNode::on_configure(const rclcpp_lifecycle::State &)
{
  step_rate_hz_ = this->get_parameter("step_rate_hz").as_double();
  stack_current_a_ = this->get_parameter("stack_current_a").as_double();
  o2_required_kg_day_ = this->get_parameter("o2_required_kg_day").as_double();
  enable_auto_faults_ = this->get_parameter("enable_auto_faults").as_bool();

  ogs::OgsParameters params = ogs::default_ogs_parameters();
  params.operating.stack_current_a = stack_current_a_;
  ogs_ = std::make_unique<ogs::OxygenGeneratorSystem>(params);

  o2_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ssos/ogs/o2_kg_day", 10);
  telemetry_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/ssos/ogs/diagnostics", 10);
  heartbeat_pub_ =
    this->create_publisher<SubsystemHeartbeat>("/ssos/ogs/heartbeat", 10);
  fault_pub_ = this->create_publisher<FaultEvent>("/ssos/fault_event", 10);
  register_client_ =
    this->create_client<RegisterSubsystem>("/ssos/register_subsystem");
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&OgsNode::on_set_parameters, this, _1));
  return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult OgsNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  // Apply live edits to the step-used members (the editor pushes here).
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : params) {
    const std::string & n = p.get_name();
    if (n == "stack_current_a") {
      stack_current_a_ = p.as_double();
    } else if (n == "o2_required_kg_day") {
      o2_required_kg_day_ = p.as_double();
    } else if (n == "step_rate_hz") {
      step_rate_hz_ = p.as_double();
    } else if (n == "enable_auto_faults") {
      enable_auto_faults_ = p.as_bool();
    }
  }
  return result;
}

CallbackReturn OgsNode::on_activate(const rclcpp_lifecycle::State &)
{
  o2_pub_->on_activate();
  telemetry_pub_->on_activate();
  heartbeat_pub_->on_activate();
  fault_pub_->on_activate();
  first_step_ = true;
  const auto period = std::chrono::duration<double>(1.0 / std::max(step_rate_hz_, 1.0e-3));
  step_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&OgsNode::step, this));
  register_with_manager();
  return CallbackReturn::SUCCESS;
}

CallbackReturn OgsNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (step_timer_) {
    step_timer_->cancel();
    step_timer_.reset();
  }
  o2_pub_->on_deactivate();
  telemetry_pub_->on_deactivate();
  heartbeat_pub_->on_deactivate();
  fault_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn OgsNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  o2_pub_.reset();
  telemetry_pub_.reset();
  heartbeat_pub_.reset();
  fault_pub_.reset();
  register_client_.reset();
  ogs_.reset();
  return CallbackReturn::SUCCESS;
}

void OgsNode::step()
{
  const rclcpp::Time now = this->now();
  double dt = 1.0 / std::max(step_rate_hz_, 1.0e-3);
  if (!first_step_) {
    const double measured = (now - last_step_time_).seconds();
    if (measured > 0.0 && measured < 100.0) {
      dt = measured;
    }
  }
  first_step_ = false;
  last_step_time_ = now;

  last_result_ = ogs_->step(dt, stack_current_a_, 1.0e9);

  std_msgs::msg::Float64 o2;
  o2.data = last_result_.o2_production_kg_day;
  o2_pub_->publish(o2);

  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = now;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "ogs";
  status.hardware_id = "ssos_eclss/ogs";
  auto kv = [&](const std::string & k, double v) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = k;
    pair.value = std::to_string(v);
    status.values.push_back(pair);
  };
  kv("o2_kg_day", last_result_.o2_production_kg_day);
  kv("h2_mol_s", last_result_.h2_production_mol_s);
  kv("stack_voltage", last_result_.stack_voltage);
  kv("stack_power_w", last_result_.stack_power_w);
  kv("stack_temp_k", last_result_.stack_temperature_k);
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "nominal";
  diag.status.push_back(status);
  telemetry_pub_->publish(diag);

  bool healthy = true;
  std::string msg = "nominal";
  if (enable_auto_faults_ && last_result_.o2_production_kg_day < o2_required_kg_day_) {
    healthy = false;
    msg = "O2 production below requirement";
    fault_pub_->publish(EclssDiagnostics::make_fault(
      now, "ogs", "o2_production_low", FaultEvent::SEVERITY_CRITICAL, msg,
      {"/ssos/ogs/o2_kg_day"}));
  }
  heartbeat_pub_->publish(EclssDiagnostics::make_heartbeat(
    now, "ogs", SubsystemHeartbeat::LIFECYCLE_ACTIVE, healthy, msg));
}

void OgsNode::register_with_manager()
{
  if (!register_client_->wait_for_service(std::chrono::milliseconds(200))) {
    RCLCPP_WARN(get_logger(), "system_manager unavailable; continuing");
    return;
  }
  auto req = std::make_shared<RegisterSubsystem::Request>();
  req->subsystem_name = "ogs";
  req->published_topics = {"/ssos/ogs/o2_kg_day", "/ssos/ogs/diagnostics"};
  req->subscribed_topics = {"/sim/world_state"};
  req->heartbeat_topic = "/ssos/ogs/heartbeat";
  register_client_->async_send_request(req);
}

}  // namespace nodes
}  // namespace ssos_eclss
