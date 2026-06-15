#include "ssos_eclss/nodes/ars_node.hpp"

#include <functional>
#include <vector>

#include "ssos_eclss/common/gas_properties.hpp"
#include "ssos_eclss/common/units.hpp"
#include "ssos_eclss/nodes/eclss_diagnostics.hpp"

using std::placeholders::_1;

namespace ssos_eclss
{
namespace nodes
{

ArsNode::ArsNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("ars_node", options)
{
  // use_sim_time defaults declared by the node; allow override.
  if (!this->has_parameter("step_rate_hz")) {
    this->declare_parameter("step_rate_hz", step_rate_hz_);
  }
  if (!this->has_parameter("co2_required_kg_day")) {
    this->declare_parameter("co2_required_kg_day", co2_required_kg_day_);
  }
}

CallbackReturn ArsNode::on_configure(const rclcpp_lifecycle::State &)
{
  step_rate_hz_ = this->get_parameter("step_rate_hz").as_double();
  co2_required_kg_day_ = this->get_parameter("co2_required_kg_day").as_double();

  bridge_ = std::make_unique<EclssParameterBridge>(this);
  bridge_->declare_ars_parameters(ars::default_ars_parameters());
  const ars::ArsParameters params = bridge_->read_ars_parameters();
  ars_ = std::make_unique<ars::FourBedSystem>(params);

  // Default cabin conditions until the first world-state arrives.
  cabin_.co2_partial_pressure_pa = units::torr_to_pa(params.operating.inlet_ppco2_torr);
  cabin_.h2o_partial_pressure_pa =
    gas::water_pp_from_rh(0.40, params.operating.cabin_temp_k);
  cabin_.temperature_k = params.operating.cabin_temp_k;
  cabin_.total_pressure_pa = params.operating.cabin_pressure_pa;

  // Publishers.
  co2_removal_pub_ =
    this->create_publisher<std_msgs::msg::Float64>("/ssos/ars/co2_removal_kg_day", 10);
  telemetry_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/ssos/ars/diagnostics", 10);
  heartbeat_pub_ =
    this->create_publisher<SubsystemHeartbeat>("/ssos/ars/heartbeat", 10);
  fault_pub_ = this->create_publisher<FaultEvent>("/ssos/fault_event", 10);

  // Subscriber to the simulation world state.
  world_state_sub_ = this->create_subscription<WorldState>(
    "/sim/world_state", 10, std::bind(&ArsNode::on_world_state, this, _1));

  // Registration client.
  register_client_ =
    this->create_client<RegisterSubsystem>("/ssos/register_subsystem");

  // Live parameter validation/apply.
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ArsNode::on_set_parameters, this, _1));

  RCLCPP_INFO(get_logger(), "ARS configured (design CO2 removal %.2f kg/day)",
              ars_->design_co2_removal_kg_day());
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArsNode::on_activate(const rclcpp_lifecycle::State &)
{
  co2_removal_pub_->on_activate();
  telemetry_pub_->on_activate();
  heartbeat_pub_->on_activate();
  fault_pub_->on_activate();

  first_step_ = true;
  const auto period = std::chrono::duration<double>(1.0 / std::max(step_rate_hz_, 1.0e-3));
  step_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ArsNode::step, this));

  register_with_manager();
  RCLCPP_INFO(get_logger(), "ARS activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArsNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (step_timer_) {
    step_timer_->cancel();
    step_timer_.reset();
  }
  co2_removal_pub_->on_deactivate();
  telemetry_pub_->on_deactivate();
  heartbeat_pub_->on_deactivate();
  fault_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArsNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  co2_removal_pub_.reset();
  telemetry_pub_.reset();
  heartbeat_pub_.reset();
  fault_pub_.reset();
  world_state_sub_.reset();
  register_client_.reset();
  ars_.reset();
  bridge_.reset();
  return CallbackReturn::SUCCESS;
}

void ArsNode::on_world_state(const WorldState::SharedPtr msg)
{
  const double total_p = units::kpa_to_pa(msg->cabin_pressure_kpa);
  cabin_.total_pressure_pa = total_p;
  cabin_.temperature_k = units::celsius_to_kelvin(msg->cabin_temp_celsius);
  cabin_.co2_partial_pressure_pa =
    units::ppm_to_fraction(msg->atmospheric_co2_ppm) * total_p;
  cabin_.h2o_partial_pressure_pa =
    gas::water_pp_from_rh(0.40, cabin_.temperature_k);
  have_world_state_ = true;
}

void ArsNode::step()
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

  // Apply any committed dynamic parameter changes before stepping.
  if (params_dirty_) {
    ars_->set_parameters(bridge_->read_ars_parameters());
    params_dirty_ = false;
  }

  last_result_ = ars_->step(dt, cabin_);

  // CO2 removal rate.
  std_msgs::msg::Float64 removal;
  removal.data = last_result_.co2_removal_rate_kg_day;
  co2_removal_pub_->publish(removal);

  // Telemetry.
  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = now;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "ars";
  status.hardware_id = "ssos_eclss/ars";
  auto kv = [&](const std::string & k, double v) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = k;
    pair.value = std::to_string(v);
    status.values.push_back(pair);
  };
  kv("co2_removal_kg_day", last_result_.co2_removal_rate_kg_day);
  kv("scrubbed_co2_torr", units::pa_to_torr(last_result_.scrubbed_co2_pp_pa));
  kv("system_dp_in_h2o", units::pa_to_in_h2o(last_result_.system_pressure_drop_pa));
  kv("max_bed_temp_k", last_result_.max_bed_temp_k);
  kv("blower_flow_scfm", last_result_.blower_flow_scfm);
  kv("precooler_exit_k", last_result_.precooler_exit_temp_k);
  kv("adsorbing_train", last_result_.adsorbing_train);
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "nominal";
  diag.status.push_back(status);
  telemetry_pub_->publish(diag);

  // Fault detection.
  bool healthy = true;
  std::string health_msg = "nominal";
  if (EclssDiagnostics::ars_co2_removal_low(last_result_.co2_removal_rate_kg_day,
                                            co2_required_kg_day_)) {
    healthy = false;
    health_msg = "CO2 removal below requirement";
    fault_pub_->publish(EclssDiagnostics::make_fault(
      now, "ars", "co2_removal_below_requirement",
      FaultEvent::SEVERITY_CRITICAL, health_msg, {"/ssos/ars/co2_removal_kg_day"}));
  }

  // Heartbeat.
  heartbeat_pub_->publish(EclssDiagnostics::make_heartbeat(
    now, "ars", SubsystemHeartbeat::LIFECYCLE_ACTIVE, healthy, health_msg));
}

void ArsNode::register_with_manager()
{
  if (!register_client_->wait_for_service(std::chrono::milliseconds(200))) {
    RCLCPP_WARN(get_logger(),
                "system_manager registration service unavailable; continuing");
    return;
  }
  auto req = std::make_shared<RegisterSubsystem::Request>();
  req->subsystem_name = "ars";
  req->published_topics = {"/ssos/ars/co2_removal_kg_day", "/ssos/ars/diagnostics"};
  req->subscribed_topics = {"/sim/world_state"};
  req->heartbeat_topic = "/ssos/ars/heartbeat";
  register_client_->async_send_request(req);
}

rcl_interfaces::msg::SetParametersResult ArsNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  // Validate first; never let a bad value reach the physics.
  rcl_interfaces::msg::SetParametersResult result = bridge_->validate(params);
  if (!result.successful) {
    RCLCPP_WARN(get_logger(), "Rejected parameter update: %s", result.reason.c_str());
    return result;
  }
  // The callback fires before values are committed, so flag a refresh that the
  // next step() performs after the new values take effect. Static (geometry)
  // changes need a reconfigure cycle to rebuild the beds.
  for (const auto & p : params) {
    if (EclssParameterBridge::is_static_parameter(p.get_name())) {
      RCLCPP_INFO(get_logger(),
                  "Static parameter '%s' changed; reconfigure to apply",
                  p.get_name().c_str());
    } else {
      params_dirty_ = true;
    }
  }
  return result;
}

}  // namespace nodes
}  // namespace ssos_eclss
