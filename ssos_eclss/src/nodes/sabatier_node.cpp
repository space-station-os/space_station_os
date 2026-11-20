#include "ssos_eclss/nodes/sabatier_node.hpp"

#include <algorithm>
#include <functional>
#include <string>

#include "ssos_eclss/common/units.hpp"
#include "ssos_eclss/nodes/eclss_diagnostics.hpp"

namespace ssos_eclss
{
namespace nodes
{

SabatierNode::SabatierNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sabatier_node", options),
  drills_(this, "sabatier"),
  health_(this, "sabatier",
    {"/ssos/sabatier/water_kg_day", "/ssos/sabatier/diagnostics"},
    {"/ssos/ars/co2_removal_kg_day", "/ssos/ogs/o2_kg_day"})
{
  this->declare_parameter("step_rate_hz", step_rate_hz_);
  this->declare_parameter("min_reactor_temp_k", min_reactor_temp_k_);
  this->declare_parameter("enable_auto_faults", enable_auto_faults_);
  if (!this->has_parameter("eps_coupling")) {
    this->declare_parameter("eps_coupling", eps_coupling_);
  }
  autostart_timer_ = EclssDiagnostics::maybe_autostart(this);
}

CallbackReturn SabatierNode::on_configure(const rclcpp_lifecycle::State &)
{
  step_rate_hz_ = this->get_parameter("step_rate_hz").as_double();
  min_reactor_temp_k_ = this->get_parameter("min_reactor_temp_k").as_double();
  enable_auto_faults_ = this->get_parameter("enable_auto_faults").as_bool();

  sabatier_ = std::make_unique<sabatier::SabatierSystem>(
    sabatier::default_sabatier_parameters());
  last_result_ = {};
  stepped_ = false;
  last_result_.reactor_temp_k = sabatier::default_sabatier_parameters().reactor.operating_temp_k;

  water_pub_ =
    this->create_publisher<std_msgs::msg::Float64>("/ssos/sabatier/water_kg_day", 10);
  telemetry_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/ssos/sabatier/diagnostics", 10);
  health_.configure();
  drills_.start();
  eps_coupling_ = this->get_parameter("eps_coupling").as_bool();
  power_.start(eps_coupling_);

  co2_in_.subscribe(this, "/ssos/ars/co2_removal_kg_day");
  ogs_o2_.subscribe(this, "/ssos/ogs/o2_kg_day");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SabatierNode::on_activate(const rclcpp_lifecycle::State &)
{
  water_pub_->on_activate();
  telemetry_pub_->on_activate();
  health_.activate();
  clock_.reset();
  const auto period = std::chrono::duration<double>(1.0 / std::max(step_rate_hz_, 1.0e-3));
  step_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SabatierNode::step, this));
  return CallbackReturn::SUCCESS;
}

CallbackReturn SabatierNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  water_pub_->on_deactivate();
  telemetry_pub_->on_deactivate();
  health_.deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SabatierNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  co2_in_.reset();
  ogs_o2_.reset();
  water_pub_.reset();
  telemetry_pub_.reset();
  health_.cleanup();
  drills_.stop();
  power_.stop();
  sabatier_.reset();
  return CallbackReturn::SUCCESS;
}

void SabatierNode::step()
{
  const rclcpp::Time now = this->now();
  drills_.update(now);
  const bool failed = drills_.active("sabatier_reactor_failure");
  const double co2_in_mol_s = co2_in_.get() / units::M_CO2;
  // PEM electrolysis gives 2 mol H2 per mol O2 (2 H2O -> 2 H2 + O2).
  const double h2_in_mol_s = 2.0 * ogs_o2_.get() / units::M_O2;
  // The reactor needs its heaters; unpowered or failed, it converts nothing
  // and all CO2 from the ARS is vented.
  const double supply = power_.powered_fraction();
  const bool running = supply > 0.999 && !failed;
  const double dt = clock_.advance(now);
  if (!running) {
    last_result_.co2_consumed_mol_s = 0.0;
    last_result_.ch4_produced_mol_s = 0.0;
    last_result_.water_produced_kg_s = 0.0;
    last_result_.heater_power_w = 0.0;
  } else if (dt > 0.0) {
    last_result_ = sabatier_->step(dt, co2_in_mol_s, h2_in_mol_s);
    full_power_w_ = last_result_.heater_power_w;
    stepped_ = true;
  }
  power_.update_demand(failed ? 0.0 : full_power_w_);
  publish(now, co2_in_mol_s, supply);

  // Conversion against what the feed allows (4 H2 per CO2).
  const double reactable = std::min(co2_in_mol_s, h2_in_mol_s / 4.0);
  std::string fault, msg = last_result_.hydrogen_limited ? "H2-limited" : "nominal";
  if (!power_.powered()) {
    fault = "sabatier_power_loss";
    msg = power_.status();
  } else if (enable_auto_faults_ && (stepped_ || failed) && reactable > 1.0e-7 &&
    last_result_.co2_consumed_mol_s < 0.1 * reactable)
  {
    fault = "sabatier_no_conversion";
    msg = "Sabatier reactor not converting CO2; CO2 vented";
  } else if (enable_auto_faults_ && last_result_.reactor_temp_k < min_reactor_temp_k_) {
    fault = "sabatier_reactor_cold";
    msg = "Sabatier reactor below operating temperature";
  }
  health_.report(now, fault, msg, FaultEvent::SEVERITY_WARNING,
    {"/ssos/sabatier/water_kg_day"});
}

void SabatierNode::publish(const rclcpp::Time & now, double co2_in_mol_s, double supply)
{
  std_msgs::msg::Float64 water;
  water.data = units::kg_per_s_to_kg_per_day(last_result_.water_produced_kg_s);
  water_pub_->publish(water);

  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = now;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "sabatier";
  status.hardware_id = "ssos_eclss/sabatier";
  auto kv = [&status](const std::string & k, double v) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = k;
      pair.value = std::to_string(v);
      status.values.push_back(pair);
    };
  // What the reactor consumed goes to methanation; the rest of the ARS CO2
  // (H2-limited surplus and unconverted) is vented.
  const double to_kg_day = units::M_CO2 * units::SECONDS_PER_DAY;
  const double co2_in_kg_day = co2_in_mol_s * to_kg_day;
  const double co2_used_kg_day = last_result_.co2_consumed_mol_s * to_kg_day;
  kv("co2_available_kg_day", co2_in_kg_day);
  kv("co2_to_sabatier_kg_day", co2_used_kg_day);
  kv("co2_vented_kg_day", std::max(0.0, co2_in_kg_day - co2_used_kg_day));
  kv("co2_to_sabatier_fraction", co2_in_kg_day > 1.0e-9 ? co2_used_kg_day / co2_in_kg_day : 0.0);
  kv("water_kg_day", water.data);
  kv("ch4_kg_day",
    units::kg_per_s_to_kg_per_day(last_result_.ch4_produced_mol_s * units::M_CH4));
  kv("conversion", last_result_.conversion);
  kv("reactor_temp_k", last_result_.reactor_temp_k);
  kv("heater_power_w", last_result_.heater_power_w);
  kv("hydrogen_limited", last_result_.hydrogen_limited ? 1.0 : 0.0);
  kv("power_demand_w", full_power_w_);
  kv("power_supply_fraction", supply);
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = drills_.tracker().any() ? "drill: " + drills_.tracker().type() :
    last_result_.hydrogen_limited ? "H2-limited" : "nominal";
  diag.status.push_back(status);
  telemetry_pub_->publish(diag);
}

}  // namespace nodes
}  // namespace ssos_eclss
