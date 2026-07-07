#include "ssos_eclss/nodes/cabin_node.hpp"

#include <functional>
#include <vector>

#include "ssos_eclss/cabin/cabin_atmosphere.hpp"
#include "ssos_eclss/common/units.hpp"
#include "ssos_eclss/nodes/eclss_diagnostics.hpp"

using std::placeholders::_1;

namespace ssos_eclss
{
namespace nodes
{

CabinNode::CabinNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("cabin_node", options)
{
  this->declare_parameter("step_rate_hz", step_rate_hz_);
  this->declare_parameter("cabin_volume_m3", cabin_volume_m3_);
  this->declare_parameter("cabin_temp_c", cabin_temp_c_);
  this->declare_parameter("co2_alarm_ppm", co2_alarm_ppm_);
  this->declare_parameter("enable_auto_faults", enable_auto_faults_);
  autostart_timer_ = EclssDiagnostics::maybe_autostart(this);
}

CallbackReturn CabinNode::on_configure(const rclcpp_lifecycle::State &)
{
  step_rate_hz_ = this->get_parameter("step_rate_hz").as_double();
  cabin_volume_m3_ = this->get_parameter("cabin_volume_m3").as_double();
  cabin_temp_c_ = this->get_parameter("cabin_temp_c").as_double();
  co2_alarm_ppm_ = this->get_parameter("co2_alarm_ppm").as_double();
  enable_auto_faults_ = this->get_parameter("enable_auto_faults").as_bool();

  cabin::CabinParams cp{};
  cp.volume_m3 = cabin_volume_m3_;
  cp.temperature_k = units::celsius_to_kelvin(cabin_temp_c_);
  atmosphere_ = std::make_unique<cabin::CabinAtmosphere>(cp);
  cabin::LeakParams lp{};
  lp.nominal_area_m2 = 1.0e-7;
  lp.discharge_coeff = 0.62;
  leak_ = std::make_unique<cabin::LeakModel>(lp);

  co2_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ssos/cabin/co2_ppm", 10);
  telemetry_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/ssos/cabin/diagnostics", 10);
  heartbeat_pub_ =
    this->create_publisher<SubsystemHeartbeat>("/ssos/cabin/heartbeat", 10);
  fault_pub_ = this->create_publisher<FaultEvent>("/ssos/fault_event", 10);
  register_client_ =
    this->create_client<RegisterSubsystem>("/ssos/register_subsystem");

  // Mass-balance inputs [kg/day -> kg/s]: crew CO2/O2 (source/sink), ARS CO2
  // removal (sink) and OGS O2 (source).
  crew_co2_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/ssos/crew/co2_kg_day", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      crew_co2_kg_s_ = units::kg_per_day_to_kg_per_s(msg->data);
    });
  crew_o2_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/ssos/crew/o2_consumption_kg_day", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      crew_o2_consumption_kg_s_ = units::kg_per_day_to_kg_per_s(msg->data);
    });
  ars_removal_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/ssos/ars/co2_removal_kg_day", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      ars_co2_removal_kg_s_ = units::kg_per_day_to_kg_per_s(msg->data);
    });
  ogs_o2_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/ssos/ogs/o2_kg_day", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      ogs_o2_kg_s_ = units::kg_per_day_to_kg_per_s(msg->data);
    });
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CabinNode::on_set_parameters, this, _1));
  return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult CabinNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : params) {
    const std::string & n = p.get_name();
    if (n == "co2_alarm_ppm") {
      co2_alarm_ppm_ = p.as_double();
    } else if (n == "step_rate_hz") {
      step_rate_hz_ = p.as_double();
    } else if (n == "enable_auto_faults") {
      enable_auto_faults_ = p.as_bool();
    }
  }
  return result;
}

CallbackReturn CabinNode::on_activate(const rclcpp_lifecycle::State &)
{
  co2_pub_->on_activate();
  telemetry_pub_->on_activate();
  heartbeat_pub_->on_activate();
  fault_pub_->on_activate();
  first_step_ = true;
  const auto period = std::chrono::duration<double>(1.0 / std::max(step_rate_hz_, 1.0e-3));
  step_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&CabinNode::step, this));
  register_with_manager();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CabinNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (step_timer_) {
    step_timer_->cancel();
    step_timer_.reset();
  }
  co2_pub_->on_deactivate();
  telemetry_pub_->on_deactivate();
  heartbeat_pub_->on_deactivate();
  fault_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CabinNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  co2_pub_.reset();
  telemetry_pub_.reset();
  heartbeat_pub_.reset();
  fault_pub_.reset();
  register_client_.reset();
  crew_co2_sub_.reset();
  crew_o2_sub_.reset();
  ars_removal_sub_.reset();
  ogs_o2_sub_.reset();
  atmosphere_.reset();
  leak_.reset();
  return CallbackReturn::SUCCESS;
}

void CabinNode::step()
{
  const rclcpp::Time now = this->now();
  double dt = 1.0 / std::max(step_rate_hz_, 1.0e-3);
  if (!first_step_) {
    const double measured = (now - last_step_time_).seconds();
    // Sim clock drives dt: elapsed sim time, 0 when the clock is paused/frozen
    // (freezes the atmosphere). Capped because the cabin<->ARS CO2 feedback is
    // integrated explicitly and is unstable with a large dt + stale removal.
    if (this->get_parameter("use_sim_time").as_bool()) {
      dt = std::min(std::max(measured, 0.0), 100.0);
    } else if (measured > 0.0 && measured < 100.0) {
      dt = measured;
    }
  }
  first_step_ = false;
  last_step_time_ = now;

  // Molar mass balance [mol/s]: crew produces CO2 / consumes O2, ARS removes
  // CO2, OGS adds O2, plus leakage. Crew latent water is condensed by the CHX
  // and routed to the WRS, so it does not accumulate in the cabin here.
  const cabin::GasFlows lk = leak_->leak_flows(*atmosphere_);
  cabin::GasFlows total{};
  total.co2 = (crew_co2_kg_s_ - ars_co2_removal_kg_s_) / units::M_CO2 + lk.co2;
  total.o2 = (ogs_o2_kg_s_ - crew_o2_consumption_kg_s_) / units::M_O2 + lk.o2;
  total.n2 = lk.n2;
  total.h2o = lk.h2o;
  atmosphere_->apply_flows(dt, total);

  last_co2_ppm_ = atmosphere_->co2_ppm();

  std_msgs::msg::Float64 co2;
  co2.data = last_co2_ppm_;
  co2_pub_->publish(co2);

  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = now;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "cabin";
  status.hardware_id = "ssos_eclss/cabin";
  auto kv = [&](const std::string & k, double v) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = k;
    pair.value = std::to_string(v);
    status.values.push_back(pair);
  };
  kv("co2_ppm", last_co2_ppm_);
  kv("o2_fraction", atmosphere_->o2_fraction());
  kv("total_pressure_kpa", units::pa_to_kpa(atmosphere_->total_pressure_pa()));
  kv("relative_humidity", atmosphere_->relative_humidity());
  kv("ars_co2_removal_kg_day", units::kg_per_s_to_kg_per_day(ars_co2_removal_kg_s_));
  kv("ogs_o2_kg_day", units::kg_per_s_to_kg_per_day(ogs_o2_kg_s_));
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "nominal";
  diag.status.push_back(status);
  telemetry_pub_->publish(diag);

  bool healthy = true;
  std::string msg = "nominal";
  if (enable_auto_faults_ && last_co2_ppm_ > co2_alarm_ppm_) {
    healthy = false;
    msg = "cabin CO2 above alarm threshold";
    fault_pub_->publish(EclssDiagnostics::make_fault(
      now, "cabin", "co2_high", FaultEvent::SEVERITY_CRITICAL, msg,
      {"/ssos/cabin/co2_ppm"}));
  }
  heartbeat_pub_->publish(EclssDiagnostics::make_heartbeat(
    now, "cabin", SubsystemHeartbeat::LIFECYCLE_ACTIVE, healthy, msg));
}

void CabinNode::register_with_manager()
{
  if (!register_client_->wait_for_service(std::chrono::milliseconds(200))) {
    RCLCPP_WARN(get_logger(), "system_manager unavailable; continuing");
    return;
  }
  auto req = std::make_shared<RegisterSubsystem::Request>();
  req->subsystem_name = "cabin";
  req->published_topics = {"/ssos/cabin/co2_ppm", "/ssos/cabin/diagnostics"};
  req->subscribed_topics = {};
  req->heartbeat_topic = "/ssos/cabin/heartbeat";
  register_client_->async_send_request(req);
}

}  // namespace nodes
}  // namespace ssos_eclss
