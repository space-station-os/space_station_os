#include "ssos_eclss/nodes/sabatier_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include "ssos_eclss/common/units.hpp"
#include "ssos_eclss/nodes/eclss_diagnostics.hpp"

namespace ssos_eclss
{
namespace nodes
{

SabatierNode::SabatierNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sabatier_node", options)
{
  this->declare_parameter("step_rate_hz", step_rate_hz_);
  autostart_timer_ = EclssDiagnostics::maybe_autostart(this);
}

CallbackReturn SabatierNode::on_configure(const rclcpp_lifecycle::State &)
{
  step_rate_hz_ = this->get_parameter("step_rate_hz").as_double();

  sabatier_ = std::make_unique<sabatier::SabatierSystem>(
    sabatier::default_sabatier_parameters());

  water_pub_ =
    this->create_publisher<std_msgs::msg::Float64>("/ssos/sabatier/water_kg_day", 10);
  telemetry_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/ssos/sabatier/diagnostics", 10);
  heartbeat_pub_ =
    this->create_publisher<SubsystemHeartbeat>("/ssos/sabatier/heartbeat", 10);
  fault_pub_ = this->create_publisher<FaultEvent>("/ssos/fault_event", 10);
  register_client_ =
    this->create_client<RegisterSubsystem>("/ssos/register_subsystem");

  // CO2 available = ARS desorption stream.
  ars_co2_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/ssos/ars/co2_removal_kg_day", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      co2_available_kg_s_ = units::kg_per_day_to_kg_per_s(msg->data);
    });
  // H2 available = 2 mol H2 per mol O2 from PEM electrolysis (2H2O -> 2H2 + O2).
  ogs_o2_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/ssos/ogs/o2_kg_day", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      const double o2_mol_s = units::kg_per_day_to_kg_per_s(msg->data) / units::M_O2;
      h2_available_mol_s_ = 2.0 * o2_mol_s;
    });
  return CallbackReturn::SUCCESS;
}

CallbackReturn SabatierNode::on_activate(const rclcpp_lifecycle::State &)
{
  water_pub_->on_activate();
  telemetry_pub_->on_activate();
  heartbeat_pub_->on_activate();
  fault_pub_->on_activate();
  first_step_ = true;
  const auto period = std::chrono::duration<double>(1.0 / std::max(step_rate_hz_, 1.0e-3));
  step_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SabatierNode::step, this));
  register_with_manager();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SabatierNode::on_deactivate(const rclcpp_lifecycle::State &)
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

CallbackReturn SabatierNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  water_pub_.reset();
  telemetry_pub_.reset();
  heartbeat_pub_.reset();
  fault_pub_.reset();
  ars_co2_sub_.reset();
  ogs_o2_sub_.reset();
  register_client_.reset();
  sabatier_.reset();
  return CallbackReturn::SUCCESS;
}

void SabatierNode::step()
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

  // Feed the reactor: all CO2 the ARS is desorbing, plus the OGS H2 stream.
  const double co2_in_mol_s = co2_available_kg_s_ / units::M_CO2;
  last_result_ = sabatier_->step(dt, co2_in_mol_s, h2_available_mol_s_);

  // CO2 split: what the reactor actually consumed goes to Sabatier; the rest of
  // the desorbed stream (H2-limited surplus + unconverted) is vented to space.
  const double co2_to_sabatier_mol_s = last_result_.co2_consumed_mol_s;
  const double co2_vented_mol_s = std::max(0.0, co2_in_mol_s - co2_to_sabatier_mol_s);

  // Recovered water -> WRS.
  std_msgs::msg::Float64 water;
  water.data = units::kg_per_s_to_kg_per_day(last_result_.water_produced_kg_s);
  water_pub_->publish(water);

  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = now;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "sabatier";
  status.hardware_id = "ssos_eclss/sabatier";
  auto kv = [&](const std::string & k, double v) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = k;
    pair.value = std::to_string(v);
    status.values.push_back(pair);
  };
  const double co2_in_kg_day = units::kg_per_s_to_kg_per_day(co2_available_kg_s_);
  const double co2_to_sab_kg_day =
    units::kg_per_s_to_kg_per_day(co2_to_sabatier_mol_s * units::M_CO2);
  const double co2_vented_kg_day =
    units::kg_per_s_to_kg_per_day(co2_vented_mol_s * units::M_CO2);
  const double split_frac = (co2_in_kg_day > 1.0e-9)
                              ? (co2_to_sab_kg_day / co2_in_kg_day) : 0.0;
  kv("co2_available_kg_day", co2_in_kg_day);
  kv("co2_to_sabatier_kg_day", co2_to_sab_kg_day);
  kv("co2_vented_kg_day", co2_vented_kg_day);
  kv("co2_to_sabatier_fraction", split_frac);
  kv("water_kg_day", water.data);
  kv("ch4_kg_day",
     units::kg_per_s_to_kg_per_day(last_result_.ch4_produced_mol_s * units::M_CH4));
  kv("conversion", last_result_.conversion);
  kv("reactor_temp_k", last_result_.reactor_temp_k);
  kv("heater_power_w", last_result_.heater_power_w);
  kv("hydrogen_limited", last_result_.hydrogen_limited ? 1.0 : 0.0);
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = last_result_.hydrogen_limited ? "H2-limited" : "nominal";
  diag.status.push_back(status);
  telemetry_pub_->publish(diag);

  heartbeat_pub_->publish(EclssDiagnostics::make_heartbeat(
    now, "sabatier", SubsystemHeartbeat::LIFECYCLE_ACTIVE, true,
    last_result_.hydrogen_limited ? "H2-limited" : "nominal"));
}

void SabatierNode::register_with_manager()
{
  if (!register_client_->wait_for_service(std::chrono::milliseconds(200))) {
    RCLCPP_WARN(get_logger(), "system_manager unavailable; continuing");
    return;
  }
  auto req = std::make_shared<RegisterSubsystem::Request>();
  req->subsystem_name = "sabatier";
  req->published_topics = {"/ssos/sabatier/water_kg_day", "/ssos/sabatier/diagnostics"};
  req->subscribed_topics = {"/ssos/ars/co2_removal_kg_day", "/ssos/ogs/o2_kg_day"};
  req->heartbeat_topic = "/ssos/sabatier/heartbeat";
  register_client_->async_send_request(req);
}

}  // namespace nodes
}  // namespace ssos_eclss
