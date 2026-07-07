#include "ssos_eclss/nodes/wrs_node.hpp"

#include <functional>
#include <vector>

#include "ssos_eclss/common/units.hpp"
#include "ssos_eclss/nodes/eclss_diagnostics.hpp"

using std::placeholders::_1;

namespace ssos_eclss
{
namespace nodes
{

WrsNode::WrsNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("wrs_node", options)
{
  this->declare_parameter("step_rate_hz", step_rate_hz_);
  this->declare_parameter("potable_limit_us", potable_limit_us_);
  this->declare_parameter("wastewater_capacity_kg", wastewater_capacity_kg_);
  this->declare_parameter("potable_capacity_kg", potable_capacity_kg_);
  this->declare_parameter("initial_potable_kg", initial_potable_kg_);
  this->declare_parameter("max_urine_process_kg_day", max_urine_process_kg_day_);
  this->declare_parameter("upa_start_fraction", upa_start_fraction_);
  this->declare_parameter("upa_stop_fraction", upa_stop_fraction_);
  this->declare_parameter("potable_reserve_kg", potable_reserve_kg_);
  this->declare_parameter("enable_auto_faults", enable_auto_faults_);
  autostart_timer_ = EclssDiagnostics::maybe_autostart(this);
}

CallbackReturn WrsNode::on_configure(const rclcpp_lifecycle::State &)
{
  step_rate_hz_ = this->get_parameter("step_rate_hz").as_double();
  potable_limit_us_ = this->get_parameter("potable_limit_us").as_double();
  wastewater_capacity_kg_ = this->get_parameter("wastewater_capacity_kg").as_double();
  potable_capacity_kg_ = this->get_parameter("potable_capacity_kg").as_double();
  initial_potable_kg_ = this->get_parameter("initial_potable_kg").as_double();
  max_urine_process_kg_day_ = this->get_parameter("max_urine_process_kg_day").as_double();
  upa_start_fraction_ = this->get_parameter("upa_start_fraction").as_double();
  upa_stop_fraction_ = this->get_parameter("upa_stop_fraction").as_double();
  potable_reserve_kg_ = this->get_parameter("potable_reserve_kg").as_double();
  enable_auto_faults_ = this->get_parameter("enable_auto_faults").as_bool();

  wrs_ = std::make_unique<wrs::WaterRecoverySystem>();
  wrs_->reset();
  potable_kg_ = initial_potable_kg_;
  wastewater_kg_ = 0.0;
  upa_processing_ = false;

  water_pub_ =
    this->create_publisher<std_msgs::msg::Float64>("/ssos/wrs/potable_kg_day", 10);
  potable_available_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/ssos/wrs/potable_available_kg", 10);
  wastewater_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/ssos/wrs/wastewater_kg", 10);
  telemetry_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/ssos/wrs/diagnostics", 10);
  heartbeat_pub_ =
    this->create_publisher<SubsystemHeartbeat>("/ssos/wrs/heartbeat", 10);
  fault_pub_ = this->create_publisher<FaultEvent>("/ssos/fault_event", 10);
  register_client_ =
    this->create_client<RegisterSubsystem>("/ssos/register_subsystem");

  auto latch = [this](const char * topic, double & target) {
    return this->create_subscription<std_msgs::msg::Float64>(
      topic, 10, [&target](const std_msgs::msg::Float64::SharedPtr msg) {
        target = units::kg_per_day_to_kg_per_s(msg->data);
      });
  };
  crew_urine_sub_ = latch("/ssos/crew/urine_kg_day", crew_urine_kg_s_);
  crew_latent_sub_ = latch("/ssos/crew/latent_water_kg_day", crew_latent_kg_s_);
  crew_potable_sub_ = latch("/ssos/crew/potable_demand_kg_day", crew_potable_demand_kg_s_);
  ogs_demand_sub_ = latch("/ssos/ogs/water_demand_kg_day", ogs_demand_kg_s_);
  sabatier_water_sub_ = latch("/ssos/sabatier/water_kg_day", sabatier_water_kg_s_);
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&WrsNode::on_set_parameters, this, _1));
  return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult WrsNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : params) {
    const std::string & n = p.get_name();
    if (n == "potable_limit_us") {
      potable_limit_us_ = p.as_double();
    } else if (n == "step_rate_hz") {
      step_rate_hz_ = p.as_double();
    } else if (n == "wastewater_capacity_kg") {
      wastewater_capacity_kg_ = p.as_double();
    } else if (n == "potable_capacity_kg") {
      potable_capacity_kg_ = p.as_double();
    } else if (n == "max_urine_process_kg_day") {
      max_urine_process_kg_day_ = p.as_double();
    } else if (n == "upa_start_fraction") {
      upa_start_fraction_ = p.as_double();
    } else if (n == "upa_stop_fraction") {
      upa_stop_fraction_ = p.as_double();
    } else if (n == "potable_reserve_kg") {
      potable_reserve_kg_ = p.as_double();
    } else if (n == "enable_auto_faults") {
      enable_auto_faults_ = p.as_bool();
    }
  }
  return result;
}

CallbackReturn WrsNode::on_activate(const rclcpp_lifecycle::State &)
{
  water_pub_->on_activate();
  potable_available_pub_->on_activate();
  wastewater_pub_->on_activate();
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
  potable_available_pub_->on_deactivate();
  wastewater_pub_->on_deactivate();
  telemetry_pub_->on_deactivate();
  heartbeat_pub_->on_deactivate();
  fault_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn WrsNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  water_pub_.reset();
  potable_available_pub_.reset();
  wastewater_pub_.reset();
  telemetry_pub_.reset();
  heartbeat_pub_.reset();
  fault_pub_.reset();
  crew_urine_sub_.reset();
  crew_latent_sub_.reset();
  crew_potable_sub_.reset();
  ogs_demand_sub_.reset();
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
    // Sim clock drives dt: elapsed sim time, 0 when the clock is paused/frozen
    // (freezes the tanks); tank integration is linear and clamped.
    if (this->get_parameter("use_sim_time").as_bool()) {
      dt = std::min(std::max(measured, 0.0), 1.0e6);
    } else if (measured > 0.0 && measured < 1.0e6) {
      dt = measured;
    }
  }
  first_step_ = false;
  last_step_time_ = now;

  // Urine (incl. flush) collects in the wastewater tank; the UPA runs as a
  // batch when the tank passes the start fraction and stops near empty (mirrors
  // the ISS WSTA 70% trigger).
  wastewater_kg_ += crew_urine_kg_s_ * dt;
  wastewater_kg_ = std::clamp(wastewater_kg_, 0.0, wastewater_capacity_kg_);
  const double ww_frac = (wastewater_capacity_kg_ > 0.0)
                           ? wastewater_kg_ / wastewater_capacity_kg_ : 0.0;
  if (!upa_processing_ && ww_frac >= upa_start_fraction_) {
    upa_processing_ = true;
  } else if (upa_processing_ && ww_frac <= upa_stop_fraction_) {
    upa_processing_ = false;
  }
  double urine_proc = 0.0;
  if (upa_processing_) {
    const double max_rate = units::kg_per_day_to_kg_per_s(max_urine_process_kg_day_);
    urine_proc = std::min(max_rate, wastewater_kg_ / dt);
    wastewater_kg_ -= urine_proc * dt;
  }

  // WPA polishes UPA distillate + humidity condensate + Sabatier product water.
  const double condensate = crew_latent_kg_s_ + sabatier_water_kg_s_;
  last_result_ = wrs_->step(dt, urine_proc, condensate, 1.0e-8);

  // Potable tank: recovered water in, crew + OGS draws out.
  potable_kg_ += last_result_.potable_water_kg_s * dt;
  const double demand = crew_potable_demand_kg_s_ + ogs_demand_kg_s_;
  const double served = std::min(demand, std::max(0.0, potable_kg_) / dt);
  potable_kg_ -= served * dt;
  potable_kg_ = std::clamp(potable_kg_, 0.0, potable_capacity_kg_);

  std_msgs::msg::Float64 water;
  water.data = last_result_.potable_water_kg_s * units::SECONDS_PER_DAY;
  water_pub_->publish(water);
  std_msgs::msg::Float64 avail;
  avail.data = potable_kg_;
  potable_available_pub_->publish(avail);
  std_msgs::msg::Float64 ww;
  ww.data = wastewater_kg_;
  wastewater_pub_->publish(ww);

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
  const double potable_demand_kg_day =
    units::kg_per_s_to_kg_per_day(crew_potable_demand_kg_s_ + ogs_demand_kg_s_);
  const double days_supply = (potable_demand_kg_day > 1.0e-6)
                               ? potable_kg_ / potable_demand_kg_day : 0.0;
  kv("potable_kg_day", water.data);
  kv("conductivity_us", last_result_.product_conductivity_us);
  kv("overall_recovery", last_result_.overall_recovery);
  kv("voc_conversion", last_result_.voc_conversion);
  kv("sabatier_feed_kg_day", units::kg_per_s_to_kg_per_day(sabatier_water_kg_s_));
  kv("potable_available_kg", potable_kg_);
  kv("wastewater_kg", wastewater_kg_);
  kv("days_of_supply", days_supply);
  kv("crew_demand_kg_day", units::kg_per_s_to_kg_per_day(crew_potable_demand_kg_s_));
  kv("ogs_demand_kg_day", units::kg_per_s_to_kg_per_day(ogs_demand_kg_s_));
  kv("upa_processing", upa_processing_ ? 1.0 : 0.0);
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "nominal";
  diag.status.push_back(status);
  telemetry_pub_->publish(diag);

  bool healthy = true;
  std::string msg = "nominal";
  if (enable_auto_faults_ &&
      (last_result_.product_conductivity_us > potable_limit_us_ ||
       last_result_.multifiltration_broken_through)) {
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
  req->published_topics = {"/ssos/wrs/potable_kg_day", "/ssos/wrs/potable_available_kg",
                           "/ssos/wrs/wastewater_kg", "/ssos/wrs/diagnostics"};
  req->subscribed_topics = {"/ssos/crew/urine_kg_day", "/ssos/crew/latent_water_kg_day",
                            "/ssos/crew/potable_demand_kg_day",
                            "/ssos/ogs/water_demand_kg_day", "/ssos/sabatier/water_kg_day"};
  req->heartbeat_topic = "/ssos/wrs/heartbeat";
  register_client_->async_send_request(req);
}

}  // namespace nodes
}  // namespace ssos_eclss
