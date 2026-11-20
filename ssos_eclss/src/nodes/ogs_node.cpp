#include "ssos_eclss/nodes/ogs_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

#include "ssos_eclss/common/units.hpp"
#include "ssos_eclss/nodes/eclss_diagnostics.hpp"

using std::placeholders::_1;

namespace ssos_eclss
{
namespace nodes
{

namespace
{
constexpr double kMaxStackStepS = 30.0;   // stack thermal update stays stable
constexpr double kCabinTimeoutS = 5.0;    // wall time before the ppO2 reading is stale
}

OgsNode::OgsNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("ogs_node", options),
  drills_(this, "ogs"),
  health_(this, "ogs",
    {"/ssos/ogs/o2_kg_day", "/ssos/ogs/water_demand_kg_day", "/ssos/ogs/diagnostics"},
    {"/ssos/cabin/atmosphere", "/ssos/wrs/potable_available_kg"})
{
  const auto c = ogs::default_o2_controller_params();
  this->declare_parameter("step_rate_hz", step_rate_hz_);
  this->declare_parameter("stack_current_a", initial_current_a_);
  this->declare_parameter("o2_setpoint_kpa", c.setpoint_kpa);
  this->declare_parameter("o2_kp_a_per_kpa", c.kp_a_per_kpa);
  this->declare_parameter("o2_integral_time_s", c.integral_time_s);
  this->declare_parameter("max_current_a", c.max_current_a);
  this->declare_parameter("enable_auto_faults", enable_auto_faults_);
  if (!this->has_parameter("eps_coupling")) {
    this->declare_parameter("eps_coupling", eps_coupling_);
  }
  autostart_timer_ = EclssDiagnostics::maybe_autostart(this);
}

CallbackReturn OgsNode::on_configure(const rclcpp_lifecycle::State &)
{
  step_rate_hz_ = this->get_parameter("step_rate_hz").as_double();
  initial_current_a_ = this->get_parameter("stack_current_a").as_double();
  enable_auto_faults_ = this->get_parameter("enable_auto_faults").as_bool();

  ogs::O2ControllerParams cp = ogs::default_o2_controller_params();
  cp.setpoint_kpa = this->get_parameter("o2_setpoint_kpa").as_double();
  cp.kp_a_per_kpa = this->get_parameter("o2_kp_a_per_kpa").as_double();
  cp.integral_time_s = this->get_parameter("o2_integral_time_s").as_double();
  cp.max_current_a = this->get_parameter("max_current_a").as_double();
  controller_.set_params(cp);
  controller_.reset(initial_current_a_);

  ogs_ = std::make_unique<ogs::OxygenGeneratorSystem>(ogs::default_ogs_parameters());
  ogs_->settle(initial_current_a_);
  // Outputs and power demand at the settled operating point, so the first
  // telemetry and the first EPS request are already right.
  last_result_ = ogs_->step(1.0e-3, initial_current_a_, 1.0e9);
  full_power_w_ = last_result_.stack_power_w;
  commanded_a_ = initial_current_a_;
  cabin_o2_kpa_ = -1.0;
  potable_kg_ = -1.0;

  o2_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ssos/ogs/o2_kg_day", 10);
  water_demand_pub_ =
    this->create_publisher<std_msgs::msg::Float64>("/ssos/ogs/water_demand_kg_day", 10);
  telemetry_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/ssos/ogs/diagnostics", 10);
  health_.configure();
  drills_.start();
  cabin_sub_ = this->create_subscription<space_station_interfaces::msg::CabinAtmosphere>(
    "/ssos/cabin/atmosphere", 10,
    [this](const space_station_interfaces::msg::CabinAtmosphere::SharedPtr msg) {
      cabin_o2_kpa_ = msg->o2_partial_pressure_kpa;
      cabin_stamp_ = wall_.now();
    });
  potable_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/ssos/wrs/potable_available_kg", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {potable_kg_ = msg->data;});
  eps_coupling_ = this->get_parameter("eps_coupling").as_bool();
  power_.start(eps_coupling_);
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&OgsNode::on_set_parameters, this, _1));
  return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult OgsNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  ogs::O2ControllerParams cp = controller_.params();
  for (const auto & p : params) {
    const std::string & n = p.get_name();
    if (n == "o2_setpoint_kpa") {
      cp.setpoint_kpa = p.as_double();
    } else if (n == "o2_kp_a_per_kpa") {
      cp.kp_a_per_kpa = p.as_double();
    } else if (n == "o2_integral_time_s") {
      cp.integral_time_s = p.as_double();
    } else if (n == "max_current_a") {
      cp.max_current_a = p.as_double();
    } else if (n == "enable_auto_faults") {
      enable_auto_faults_ = p.as_bool();
    }
  }
  controller_.set_params(cp);
  return result;
}

CallbackReturn OgsNode::on_activate(const rclcpp_lifecycle::State &)
{
  o2_pub_->on_activate();
  water_demand_pub_->on_activate();
  telemetry_pub_->on_activate();
  health_.activate();
  clock_.reset();
  const auto period = std::chrono::duration<double>(1.0 / std::max(step_rate_hz_, 1.0e-3));
  step_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&OgsNode::step, this));
  return CallbackReturn::SUCCESS;
}

CallbackReturn OgsNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  o2_pub_->on_deactivate();
  water_demand_pub_->on_deactivate();
  telemetry_pub_->on_deactivate();
  health_.deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn OgsNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  cabin_sub_.reset();
  potable_sub_.reset();
  o2_pub_.reset();
  water_demand_pub_.reset();
  telemetry_pub_.reset();
  health_.cleanup();
  drills_.stop();
  power_.stop();
  ogs_.reset();
  return CallbackReturn::SUCCESS;
}

void OgsNode::step()
{
  const rclcpp::Time now = this->now();
  drills_.update(now);
  const bool failed = drills_.active("ogs_generation_failure");
  // Electrolysis rate follows the current EPS can deliver.
  const double supply = power_.powered_fraction();
  const double dt = clock_.advance(now);
  if (dt > 0.0) {
    // Without a current cabin reading, hold the current where it is.
    const bool cabin_fresh = cabin_o2_kpa_ >= 0.0 &&
      (wall_.now() - cabin_stamp_).seconds() < kCabinTimeoutS;
    const double measured = cabin_fresh ? cabin_o2_kpa_ : controller_.params().setpoint_kpa;
    // Without WRS telemetry the stack is not feedwater-limited.
    const double water_mol_s = potable_kg_ >= 0.0 ?
      std::max(potable_kg_, 0.0) / units::M_H2O / dt : 1.0e9;
    const int n = std::max(1, static_cast<int>(std::ceil(dt / kMaxStackStepS)));
    const double h = dt / n;
    for (int i = 0; i < n; ++i) {
      // The integrator holds while the stack cannot follow it (failed, or EPS
      // not delivering the full current), so it does not wind up.
      const bool following = supply > 0.999 && !failed;
      commanded_a_ = (following ? controller_.update(h, measured) : controller_.current()) *
        supply;
      last_result_ = ogs_->step(h, failed ? 0.0 : commanded_a_, water_mol_s);
    }
    if (supply > 0.999 && !failed) {
      full_power_w_ = last_result_.stack_power_w;
    }
  }
  power_.update_demand(failed ? 0.0 : full_power_w_);
  publish(now, supply);

  // Faraday: O2 the commanded current should make, before separator losses.
  const double n_cells = ogs_->parameters().stack.n_cells;
  const double expected_kg_day =
    commanded_a_ * n_cells / (4.0 * units::FARADAY) * units::M_O2 * units::SECONDS_PER_DAY;
  std::string fault, msg = "nominal";
  if (!power_.powered()) {
    fault = "ogs_power_loss";
    msg = power_.status();
  } else if (enable_auto_faults_ && commanded_a_ > 2.0 &&
    last_result_.o2_production_kg_day < 0.5 * expected_kg_day)
  {
    fault = "o2_generation_low";
    msg = "O2 production far below the commanded stack current";
  } else if (enable_auto_faults_ && last_result_.feedwater_limited) {
    fault = "ogs_feedwater_low";
    msg = "OGS feedwater limited by potable inventory";
  }
  health_.report(now, fault, msg, FaultEvent::SEVERITY_CRITICAL, {"/ssos/ogs/o2_kg_day"});
}

void OgsNode::publish(const rclcpp::Time & now, double supply)
{
  std_msgs::msg::Float64 f;
  f.data = last_result_.o2_production_kg_day;
  o2_pub_->publish(f);
  f.data = last_result_.water_consumed_kg_day;
  water_demand_pub_->publish(f);

  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = now;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "ogs";
  status.hardware_id = "ssos_eclss/ogs";
  auto kv = [&status](const std::string & k, double v) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = k;
      pair.value = std::to_string(v);
      status.values.push_back(pair);
    };
  kv("o2_kg_day", last_result_.o2_production_kg_day);
  kv("commanded_current_a", commanded_a_);
  kv("cabin_o2_kpa", cabin_o2_kpa_);
  kv("o2_setpoint_kpa", controller_.params().setpoint_kpa);
  kv("h2_mol_s", last_result_.h2_production_mol_s);
  kv("water_demand_kg_day", last_result_.water_consumed_kg_day);
  kv("feedwater_limited", last_result_.feedwater_limited ? 1.0 : 0.0);
  kv("stack_voltage", last_result_.stack_voltage);
  kv("stack_power_w", last_result_.stack_power_w);
  kv("stack_temp_k", last_result_.stack_temperature_k);
  kv("power_demand_w", full_power_w_);
  kv("power_supply_fraction", supply);
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = drills_.tracker().any() ? "drill: " + drills_.tracker().type() : "nominal";
  diag.status.push_back(status);
  telemetry_pub_->publish(diag);
}

}  // namespace nodes
}  // namespace ssos_eclss
