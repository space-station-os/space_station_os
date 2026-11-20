#include "ssos_eclss/nodes/wrs_node.hpp"

#include <algorithm>
#include <functional>
#include <string>

#include "ssos_eclss/common/units.hpp"
#include "ssos_eclss/nodes/eclss_diagnostics.hpp"

using std::placeholders::_1;

namespace ssos_eclss
{
namespace nodes
{

WrsNode::WrsNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("wrs_node", options),
  drills_(this, "wrs"),
  health_(this, "wrs",
    {"/ssos/wrs/potable_kg_day", "/ssos/wrs/potable_available_kg", "/ssos/wrs/wastewater_kg",
      "/ssos/wrs/diagnostics"},
    {"/ssos/crew/urine_kg_day", "/ssos/crew/potable_demand_kg_day",
      "/ssos/cabin/condensate_kg_day", "/ssos/sabatier/water_kg_day",
      "/ssos/ogs/water_demand_kg_day"})
{
  this->declare_parameter("step_rate_hz", step_rate_hz_);
  this->declare_parameter("potable_limit_us", potable_limit_us_);
  this->declare_parameter("wastewater_capacity_kg", wastewater_capacity_kg_);
  this->declare_parameter("potable_capacity_kg", potable_capacity_kg_);
  this->declare_parameter("initial_potable_kg", initial_potable_kg_);
  this->declare_parameter("initial_wastewater_kg", initial_wastewater_kg_);
  this->declare_parameter("max_urine_process_kg_day", max_urine_process_kg_day_);
  this->declare_parameter("upa_start_fraction", upa_start_fraction_);
  this->declare_parameter("upa_stop_fraction", upa_stop_fraction_);
  this->declare_parameter("low_supply_days", low_supply_days_);
  this->declare_parameter("wpa_reprocess_kg_day", wpa_reprocess_kg_day_);
  this->declare_parameter("wpa_waste_capacity_kg", wpa_waste_capacity_kg_);
  this->declare_parameter("enable_auto_faults", enable_auto_faults_);
  if (!this->has_parameter("eps_coupling")) {
    this->declare_parameter("eps_coupling", eps_coupling_);
  }
  autostart_timer_ = EclssDiagnostics::maybe_autostart(this);
}

CallbackReturn WrsNode::on_configure(const rclcpp_lifecycle::State &)
{
  step_rate_hz_ = this->get_parameter("step_rate_hz").as_double();
  potable_limit_us_ = this->get_parameter("potable_limit_us").as_double();
  wastewater_capacity_kg_ = this->get_parameter("wastewater_capacity_kg").as_double();
  potable_capacity_kg_ = this->get_parameter("potable_capacity_kg").as_double();
  initial_potable_kg_ = this->get_parameter("initial_potable_kg").as_double();
  initial_wastewater_kg_ = this->get_parameter("initial_wastewater_kg").as_double();
  max_urine_process_kg_day_ = this->get_parameter("max_urine_process_kg_day").as_double();
  upa_start_fraction_ = this->get_parameter("upa_start_fraction").as_double();
  upa_stop_fraction_ = this->get_parameter("upa_stop_fraction").as_double();
  low_supply_days_ = this->get_parameter("low_supply_days").as_double();
  wpa_reprocess_kg_day_ = this->get_parameter("wpa_reprocess_kg_day").as_double();
  wpa_waste_capacity_kg_ = this->get_parameter("wpa_waste_capacity_kg").as_double();
  enable_auto_faults_ = this->get_parameter("enable_auto_faults").as_bool();

  wrs_ = std::make_unique<wrs::WaterRecoverySystem>();
  wrs_->reset();
  last_result_ = {};
  last_result_.potable_in_spec = true;
  potable_kg_ = initial_potable_kg_;
  wastewater_kg_ = initial_wastewater_kg_;
  wpa_waste_kg_ = 0.0;
  overflow_kg_ = 0.0;
  upa_processing_ = false;
  bed_exhausted_ = false;
  urine_processed_kg_s_ = 0.0;
  off_spec_kg_s_ = 0.0;

  water_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ssos/wrs/potable_kg_day", 10);
  potable_pub_ =
    this->create_publisher<std_msgs::msg::Float64>("/ssos/wrs/potable_available_kg", 10);
  wastewater_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ssos/wrs/wastewater_kg", 10);
  telemetry_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/ssos/wrs/diagnostics", 10);
  health_.configure();
  drills_.start();
  eps_coupling_ = this->get_parameter("eps_coupling").as_bool();
  power_.start(eps_coupling_);

  urine_.subscribe(this, "/ssos/crew/urine_kg_day");
  crew_demand_.subscribe(this, "/ssos/crew/potable_demand_kg_day");
  condensate_.subscribe(this, "/ssos/cabin/condensate_kg_day");
  sabatier_water_.subscribe(this, "/ssos/sabatier/water_kg_day");
  ogs_demand_.subscribe(this, "/ssos/ogs/water_demand_kg_day");
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
    } else if (n == "max_urine_process_kg_day") {
      max_urine_process_kg_day_ = p.as_double();
    } else if (n == "upa_start_fraction") {
      upa_start_fraction_ = p.as_double();
    } else if (n == "upa_stop_fraction") {
      upa_stop_fraction_ = p.as_double();
    } else if (n == "low_supply_days") {
      low_supply_days_ = p.as_double();
    } else if (n == "enable_auto_faults") {
      enable_auto_faults_ = p.as_bool();
    }
  }
  return result;
}

CallbackReturn WrsNode::on_activate(const rclcpp_lifecycle::State &)
{
  water_pub_->on_activate();
  potable_pub_->on_activate();
  wastewater_pub_->on_activate();
  telemetry_pub_->on_activate();
  health_.activate();
  clock_.reset();
  const auto period = std::chrono::duration<double>(1.0 / std::max(step_rate_hz_, 1.0e-3));
  step_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&WrsNode::step, this));
  return CallbackReturn::SUCCESS;
}

CallbackReturn WrsNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  water_pub_->on_deactivate();
  potable_pub_->on_deactivate();
  wastewater_pub_->on_deactivate();
  telemetry_pub_->on_deactivate();
  health_.deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn WrsNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  for (auto * in : {&urine_, &crew_demand_, &condensate_, &sabatier_water_, &ogs_demand_}) {
    in->reset();
  }
  water_pub_.reset();
  potable_pub_.reset();
  wastewater_pub_.reset();
  telemetry_pub_.reset();
  health_.cleanup();
  drills_.stop();
  power_.stop();
  wrs_.reset();
  return CallbackReturn::SUCCESS;
}

void WrsNode::apply_drill()
{
  // Breakthrough exhausts the bed; when the drill ends the crew has swapped
  // in a fresh one. Other drills leave the bed alone.
  const bool exhausted = drills_.active("wpa_multifiltration_breakthrough");
  if (exhausted && !bed_exhausted_) {
    wrs_->exhaust_multifiltration();
  } else if (!exhausted && bed_exhausted_) {
    wrs_->replace_multifiltration();
  }
  bed_exhausted_ = exhausted;
}

void WrsNode::collect_urine(double dt)
{
  const double room = wastewater_capacity_kg_ - wastewater_kg_;
  const double in = urine_.get() * dt;
  wastewater_kg_ += std::min(in, room);
  overflow_kg_ += std::max(in - room, 0.0);
}

void WrsNode::hold_for_wpa(double kg)
{
  const double room = wpa_waste_capacity_kg_ - wpa_waste_kg_;
  wpa_waste_kg_ += std::min(kg, room);
  overflow_kg_ += std::max(kg - room, 0.0);
}

void WrsNode::draw_potable(double dt)
{
  const double served = std::min(crew_demand_.get() + ogs_demand_.get(), potable_kg_ / dt);
  potable_kg_ -= served * dt;
}

void WrsNode::advance(double dt)
{
  // Urine (with flush water) collects in the wastewater tank; the UPA runs a
  // batch from the start fraction down to the stop fraction, as the ISS WSTA.
  collect_urine(dt);
  const double fill = wastewater_capacity_kg_ > 0.0 ? wastewater_kg_ / wastewater_capacity_kg_ : 0.0;
  if (!upa_processing_ && fill >= upa_start_fraction_) {
    upa_processing_ = true;
  } else if (upa_processing_ && fill <= upa_stop_fraction_) {
    upa_processing_ = false;
  }
  urine_processed_kg_s_ = 0.0;
  if (upa_processing_ && !drills_.active("upa_assembly_failure")) {
    urine_processed_kg_s_ = std::min(
      units::kg_per_day_to_kg_per_s(max_urine_process_kg_day_), wastewater_kg_ / dt);
    wastewater_kg_ -= urine_processed_kg_s_ * dt;
  }

  // While the product is in spec, held-back off-spec water is fed through again.
  const double reprocess = last_result_.potable_in_spec ?
    std::min(units::kg_per_day_to_kg_per_s(wpa_reprocess_kg_day_), wpa_waste_kg_ / dt) : 0.0;
  wpa_waste_kg_ -= reprocess * dt;
  last_result_ = wrs_->step(dt, urine_processed_kg_s_,
      condensate_.get() + sabatier_water_.get() + reprocess, 1.0e-8);

  off_spec_kg_s_ = last_result_.potable_in_spec ? 0.0 : last_result_.potable_water_kg_s;
  hold_for_wpa(off_spec_kg_s_ * dt);
  potable_kg_ = std::min(potable_kg_ + (last_result_.potable_water_kg_s - off_spec_kg_s_) * dt,
      potable_capacity_kg_);
  draw_potable(dt);
}

void WrsNode::step()
{
  const rclcpp::Time now = this->now();
  if (drills_.update(now)) {
    apply_drill();
  }
  // Distillation and filtration need power; unpowered, nothing is processed.
  const double supply = power_.powered_fraction();
  const double dt = clock_.advance(now);
  if (dt > 0.0) {
    if (supply > 0.999) {
      advance(dt);
      full_power_w_ = last_result_.power_w;
    } else {
      last_result_.potable_water_kg_s = 0.0;
      urine_processed_kg_s_ = 0.0;
      off_spec_kg_s_ = 0.0;
      collect_urine(dt);
      // Condensate and Sabatier water wait for the WPA instead of vanishing.
      hold_for_wpa((condensate_.get() + sabatier_water_.get()) * dt);
      draw_potable(dt);
    }
  }
  power_.update_demand(full_power_w_);
  publish(now, supply);

  const double demand_kg_day =
    units::kg_per_s_to_kg_per_day(crew_demand_.get() + ogs_demand_.get());
  std::string fault, msg = upa_processing_ ? "UPA processing" : "nominal";
  uint8_t severity = FaultEvent::SEVERITY_CRITICAL;
  if (drills_.active("upa_assembly_failure")) {
    fault = "upa_assembly_failure";
    msg = "urine processor assembly failed";
  } else if (!power_.powered()) {
    fault = "wrs_power_loss";
    msg = power_.status();
  } else if (enable_auto_faults_) {
    if (!last_result_.potable_in_spec) {
      fault = "water_quality_out_of_spec";
      msg = "product water out of potable spec; recycling";
    } else if (wastewater_kg_ >= 0.95 * wastewater_capacity_kg_) {
      fault = "wastewater_tank_full";
      msg = "wastewater tank full";
    } else if (demand_kg_day > 0.0 && potable_kg_ / demand_kg_day < low_supply_days_) {
      fault = "potable_water_low";
      msg = "potable water below reserve";
      severity = FaultEvent::SEVERITY_WARNING;
    }
  }
  health_.report(now, fault, msg, severity, {"/ssos/wrs/potable_available_kg"});
}

void WrsNode::publish(const rclcpp::Time & now, double supply)
{
  std_msgs::msg::Float64 f;
  f.data = units::kg_per_s_to_kg_per_day(off_spec_kg_s_ > 0.0 ? 0.0 :
      last_result_.potable_water_kg_s);
  water_pub_->publish(f);
  const double potable_kg_day = f.data;
  f.data = potable_kg_;
  potable_pub_->publish(f);
  f.data = wastewater_kg_;
  wastewater_pub_->publish(f);

  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = now;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "wrs";
  status.hardware_id = "ssos_eclss/wrs";
  auto kv = [&status](const std::string & k, double v) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = k;
      pair.value = std::to_string(v);
      status.values.push_back(pair);
    };
  const double demand_kg_day =
    units::kg_per_s_to_kg_per_day(crew_demand_.get() + ogs_demand_.get());
  kv("potable_kg_day", potable_kg_day);
  kv("potable_available_kg", potable_kg_);
  kv("wastewater_kg", wastewater_kg_);
  kv("days_of_supply", demand_kg_day > 1.0e-6 ? potable_kg_ / demand_kg_day : 0.0);
  kv("upa_processing", upa_processing_ ? 1.0 : 0.0);
  kv("urine_processed_kg_day", units::kg_per_s_to_kg_per_day(urine_processed_kg_s_));
  kv("condensate_kg_day", units::kg_per_s_to_kg_per_day(condensate_.get()));
  kv("sabatier_feed_kg_day", units::kg_per_s_to_kg_per_day(sabatier_water_.get()));
  kv("crew_demand_kg_day", units::kg_per_s_to_kg_per_day(crew_demand_.get()));
  kv("ogs_demand_kg_day", units::kg_per_s_to_kg_per_day(ogs_demand_.get()));
  kv("wpa_waste_kg", wpa_waste_kg_);
  kv("wastewater_overflow_kg", overflow_kg_);
  kv("conductivity_us", last_result_.product_conductivity_us);
  kv("overall_recovery", last_result_.overall_recovery);
  kv("voc_conversion", last_result_.voc_conversion);
  kv("power_demand_w", full_power_w_);
  kv("power_supply_fraction", supply);
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = drills_.tracker().any() ? "drill: " + drills_.tracker().type() : "nominal";
  diag.status.push_back(status);
  telemetry_pub_->publish(diag);
}

}  // namespace nodes
}  // namespace ssos_eclss
