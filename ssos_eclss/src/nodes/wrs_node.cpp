#include "ssos_eclss/nodes/wrs_node.hpp"

#include <functional>

#include "ssos_eclss/common/units.hpp"
#include "ssos_eclss/nodes/eclss_diagnostics.hpp"

namespace ssos_eclss
{
namespace nodes
{

WrsNode::WrsNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("wrs_node", options)
{
  this->declare_parameter("step_rate_hz", step_rate_hz_);
  this->declare_parameter("urine_kg_day", urine_kg_day_);
  this->declare_parameter("condensate_kg_day", condensate_kg_day_);
  this->declare_parameter("potable_limit_us", potable_limit_us_);
  autostart_timer_ = EclssDiagnostics::maybe_autostart(this);
}

CallbackReturn WrsNode::on_configure(const rclcpp_lifecycle::State &)
{
  step_rate_hz_ = this->get_parameter("step_rate_hz").as_double();
  urine_kg_day_ = this->get_parameter("urine_kg_day").as_double();
  condensate_kg_day_ = this->get_parameter("condensate_kg_day").as_double();
  potable_limit_us_ = this->get_parameter("potable_limit_us").as_double();

  wrs_ = std::make_unique<wrs::WaterRecoverySystem>();
  wrs_->reset();

  water_pub_ =
    this->create_publisher<std_msgs::msg::Float64>("/ssos/wrs/potable_kg_day", 10);
  telemetry_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/ssos/wrs/diagnostics", 10);
  heartbeat_pub_ =
    this->create_publisher<SubsystemHeartbeat>("/ssos/wrs/heartbeat", 10);
  fault_pub_ = this->create_publisher<FaultEvent>("/ssos/fault_event", 10);
  register_client_ =
    this->create_client<RegisterSubsystem>("/ssos/register_subsystem");

  // Sabatier-recovered water (clean) joins the WRS feed.
  sabatier_water_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/ssos/sabatier/water_kg_day", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      sabatier_water_kg_s_ = units::kg_per_day_to_kg_per_s(msg->data);
    });
  return CallbackReturn::SUCCESS;
}

CallbackReturn WrsNode::on_activate(const rclcpp_lifecycle::State &)
{
  water_pub_->on_activate();
  telemetry_pub_->on_activate();
  heartbeat_pub_->on_activate();
  fault_pub_->on_activate();
  first_step_ = true;
  const auto period = std::chrono::duration<double>(1.0 / std::max(step_rate_hz_, 1.0e-3));
  step_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&WrsNode::step, this));
  register_with_manager();
  return CallbackReturn::SUCCESS;
}

CallbackReturn WrsNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (step_timer_) {
    step_timer_->cancel();
    step_timer_.reset();
  }
  water_pub_->on_deactivate();
  telemetry_pub_->on_deactivate();
  heartbeat_pub_->on_deactivate();
  fault_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn WrsNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  water_pub_.reset();
  telemetry_pub_.reset();
  heartbeat_pub_.reset();
  fault_pub_.reset();
  sabatier_water_sub_.reset();
  register_client_.reset();
  wrs_.reset();
  return CallbackReturn::SUCCESS;
}

void WrsNode::step()
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

  const double urine = units::kg_per_day_to_kg_per_s(urine_kg_day_);
  // Humidity condensate + Sabatier product water (both clean feeds).
  const double condensate =
    units::kg_per_day_to_kg_per_s(condensate_kg_day_) + sabatier_water_kg_s_;
  last_result_ = wrs_->step(dt, urine, condensate, 1.0e-8);

  std_msgs::msg::Float64 water;
  water.data = last_result_.potable_water_kg_s * units::SECONDS_PER_DAY;
  water_pub_->publish(water);

  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = now;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "wrs";
  status.hardware_id = "ssos_eclss/wrs";
  auto kv = [&](const std::string & k, double v) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = k;
    pair.value = std::to_string(v);
    status.values.push_back(pair);
  };
  kv("potable_kg_day", water.data);
  kv("conductivity_us", last_result_.product_conductivity_us);
  kv("overall_recovery", last_result_.overall_recovery);
  kv("voc_conversion", last_result_.voc_conversion);
  kv("sabatier_feed_kg_day", units::kg_per_s_to_kg_per_day(sabatier_water_kg_s_));
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "nominal";
  diag.status.push_back(status);
  telemetry_pub_->publish(diag);

  bool healthy = true;
  std::string msg = "nominal";
  if (last_result_.product_conductivity_us > potable_limit_us_ ||
      last_result_.multifiltration_broken_through) {
    healthy = false;
    msg = "product water out of potable spec";
    fault_pub_->publish(EclssDiagnostics::make_fault(
      now, "wrs", "water_quality_out_of_spec", FaultEvent::SEVERITY_CRITICAL, msg,
      {"/ssos/wrs/potable_kg_day"}));
  }
  heartbeat_pub_->publish(EclssDiagnostics::make_heartbeat(
    now, "wrs", SubsystemHeartbeat::LIFECYCLE_ACTIVE, healthy, msg));
}

void WrsNode::register_with_manager()
{
  if (!register_client_->wait_for_service(std::chrono::milliseconds(200))) {
    RCLCPP_WARN(get_logger(), "system_manager unavailable; continuing");
    return;
  }
  auto req = std::make_shared<RegisterSubsystem::Request>();
  req->subsystem_name = "wrs";
  req->published_topics = {"/ssos/wrs/potable_kg_day", "/ssos/wrs/diagnostics"};
  req->subscribed_topics = {"/ssos/sabatier/water_kg_day"};
  req->heartbeat_topic = "/ssos/wrs/heartbeat";
  register_client_->async_send_request(req);
}

}  // namespace nodes
}  // namespace ssos_eclss
