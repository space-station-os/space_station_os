#include "ssos_eps/nodes/eps_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "ssos_eps/nodes/eps_diagnostics.hpp"

using std::placeholders::_1;

namespace ssos_eps
{
namespace nodes
{

namespace
{
constexpr double kMaxStepS = 600.0;
}  // namespace

EpsNode::EpsNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("eps_node", options)
{
  this->declare_parameter("step_rate_hz", step_rate_hz_);
  this->declare_parameter("enable_auto_faults", enable_auto_faults_);
  this->declare_parameter("faults.failed_oru", failed_oru_);
  this->declare_parameter("faults.faulted_bcdu", faulted_bcdu_);
  this->declare_parameter("faults.faulted_ddcu", faulted_ddcu_);
  this->declare_parameter("faults.faulted_mbsu", faulted_mbsu_);
  this->declare_parameter("faults.offline_channel", offline_channel_);

  bridge_ = std::make_unique<EpsParameterBridge>(this);
  bridge_->declare_parameters(system::default_eps_parameters());

  autostart_timer_ = EpsDiagnostics::maybe_autostart(this);
}

CallbackReturn EpsNode::on_configure(const rclcpp_lifecycle::State &)
{
  step_rate_hz_ = this->get_parameter("step_rate_hz").as_double();
  enable_auto_faults_ = this->get_parameter("enable_auto_faults").as_bool();
  failed_oru_ = static_cast<int>(this->get_parameter("faults.failed_oru").as_int());
  faulted_bcdu_ = static_cast<int>(this->get_parameter("faults.faulted_bcdu").as_int());
  faulted_ddcu_ = static_cast<int>(this->get_parameter("faults.faulted_ddcu").as_int());
  faulted_mbsu_ = static_cast<int>(this->get_parameter("faults.faulted_mbsu").as_int());
  offline_channel_ = static_cast<int>(this->get_parameter("faults.offline_channel").as_int());

  eps_ = std::make_unique<system::EpsSystem>(bridge_->read_parameters());
  apply_fault_hooks();
  last_state_ = eps_->state();

  telemetry_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/ssos/eps/diagnostics", 10);
  heartbeat_pub_ = this->create_publisher<SubsystemHeartbeat>("/ssos/eps/heartbeat", 10);
  fault_pub_ = this->create_publisher<FaultEvent>("/ssos/fault_event", 10);

  bcdu_pub_ = this->create_publisher<BCDUStatus>("/ssos/eps/bcdu/status", 10);
  channel_pubs_.clear();
  for (int c = 0; c < eps_->n_channels(); ++c) {
    channel_pubs_.push_back(this->create_publisher<BCDUStatus>(
        "/ssos/eps/bcdu/channel_" + std::to_string(c) + "/status", 10));
  }
  oru_pubs_.clear();
  for (int i = 0; i < eps_->n_orus(); ++i) {
    oru_pubs_.push_back(this->create_publisher<sensor_msgs::msg::BatteryState>(
        "/ssos/eps/battery/oru_" + std::to_string(i) + "/state", 10));
  }
  mbsu_pubs_.clear();
  for (int m = 0; m < eps_->n_mbsu(); ++m) {
    mbsu_pubs_.push_back(this->create_publisher<MBSUStatus>(
        "/ssos/eps/mbsu/mbsu_" + std::to_string(m) + "/status", 10));
  }
  ddcu_pubs_.clear();
  for (int d = 0; d < eps_->n_ddcu(); ++d) {
    ddcu_pubs_.push_back(this->create_publisher<DDCUStatus>(
        "/ssos/eps/ddcu/ddcu_" + std::to_string(d) + "/status", 10));
  }
  soc_pub_ = this->create_publisher<Float64>("/ssos/eps/battery/soc", 10);
  power_balance_pub_ = this->create_publisher<Float64>("/ssos/eps/power_balance_w", 10);
  array_power_pub_ = this->create_publisher<Float64>("/ssos/eps/solar/array_power_w", 10);
  ssu_voltage_pub_ = this->create_publisher<Float64>("/ssos/eps/solar/ssu_voltage_v", 10);
  ssu_power_pub_ = this->create_publisher<Float64>("/ssos/eps/solar/ssu_power_w", 10);
  ssu_current_pub_ = this->create_publisher<Float64>("/ssos/eps/solar/ssu_current_a", 10);
  panel_temp_pub_ = this->create_publisher<Float64>(
    "/ssos/eps/solar/panel_temperature_c", 10);
  beta_pub_ = this->create_publisher<Float64>("/ssos/eps/solar/beta_deg", 10);

  fault_sub_ = this->create_subscription<FaultEvent>(
    "/ssos/fault_event", 10, std::bind(&EpsNode::on_fault_event, this, _1));
  injection_.clear();
  applied_injection_revision_ = injection_.revision();
  world_state_sub_ = this->create_subscription<WorldState>(
    "/sim/world_state", 10, std::bind(&EpsNode::on_world_state, this, _1));
  register_client_ = this->create_client<RegisterSubsystem>("/ssos/register_subsystem");
  load_srv_ = this->create_service<Load>(
    "/ssos/eps/load_request",
    std::bind(&EpsNode::handle_load_request, this, _1, std::placeholders::_2));
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&EpsNode::on_set_parameters, this, _1));

  low_soc_reported_ = false;
  load_unserved_reported_ = false;
  bcdu_fault_reported_ = false;
  oru_failed_reported_ = false;
  oru_overheat_reported_ = false;
  ddcu_fault_reported_ = false;
  ddcu_overload_reported_ = false;
  mbsu_unstable_reported_ = false;
  redundancy_reported_ = false;
  array_degraded_reported_ = false;
  battery_degraded_reported_ = false;
  params_dirty_ = false;

  RCLCPP_INFO(get_logger(),
              "Configured: %d channels x %d ORUs, %d MBSUs, %d DDCUs, %zu loads (%.0f W)",
              eps_->n_channels(), eps_->parameters().orus_per_channel, eps_->n_mbsu(),
              eps_->n_ddcu(), eps_->loads().loads().size(),
              eps_->loads().total_requested_w());
  return CallbackReturn::SUCCESS;
}

CallbackReturn EpsNode::on_activate(const rclcpp_lifecycle::State &)
{
  telemetry_pub_->on_activate();
  heartbeat_pub_->on_activate();
  fault_pub_->on_activate();
  bcdu_pub_->on_activate();
  for (auto & p : channel_pubs_) {
    p->on_activate();
  }
  for (auto & p : oru_pubs_) {
    p->on_activate();
  }
  for (auto & p : mbsu_pubs_) {
    p->on_activate();
  }
  for (auto & p : ddcu_pubs_) {
    p->on_activate();
  }
  for (auto & p : {soc_pub_, power_balance_pub_, array_power_pub_, ssu_voltage_pub_,
      ssu_power_pub_, ssu_current_pub_, panel_temp_pub_, beta_pub_})
  {
    p->on_activate();
  }

  first_step_ = true;
  const auto period = std::chrono::duration<double>(1.0 / std::max(step_rate_hz_, 1.0e-3));
  step_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&EpsNode::step, this));
  register_with_manager();
  return CallbackReturn::SUCCESS;
}

CallbackReturn EpsNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (step_timer_) {
    step_timer_->cancel();
    step_timer_.reset();
  }
  telemetry_pub_->on_deactivate();
  heartbeat_pub_->on_deactivate();
  fault_pub_->on_deactivate();
  bcdu_pub_->on_deactivate();
  for (auto & p : channel_pubs_) {
    p->on_deactivate();
  }
  for (auto & p : oru_pubs_) {
    p->on_deactivate();
  }
  for (auto & p : mbsu_pubs_) {
    p->on_deactivate();
  }
  for (auto & p : ddcu_pubs_) {
    p->on_deactivate();
  }
  for (auto & p : {soc_pub_, power_balance_pub_, array_power_pub_, ssu_voltage_pub_,
      ssu_power_pub_, ssu_current_pub_, panel_temp_pub_, beta_pub_})
  {
    p->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn EpsNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  telemetry_pub_.reset();
  heartbeat_pub_.reset();
  fault_pub_.reset();
  bcdu_pub_.reset();
  channel_pubs_.clear();
  oru_pubs_.clear();
  mbsu_pubs_.clear();
  ddcu_pubs_.clear();
  soc_pub_.reset();
  power_balance_pub_.reset();
  array_power_pub_.reset();
  ssu_voltage_pub_.reset();
  ssu_power_pub_.reset();
  ssu_current_pub_.reset();
  panel_temp_pub_.reset();
  beta_pub_.reset();
  world_state_sub_.reset();
  fault_sub_.reset();
  injection_.clear();
  register_client_.reset();
  load_srv_.reset();
  param_cb_handle_.reset();
  eps_.reset();
  return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult EpsNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result = bridge_->validate(params);
  if (!result.successful) {
    RCLCPP_WARN(get_logger(), "Rejected parameter update: %s", result.reason.c_str());
    return result;
  }
  for (const auto & p : params) {
    const std::string & n = p.get_name();
    if (n == "step_rate_hz") {
      if (p.as_double() <= 0.0) {
        result.successful = false;
        result.reason = "step_rate_hz must be > 0";
        return result;
      }
      step_rate_hz_ = p.as_double();
    } else if (n == "enable_auto_faults") {
      enable_auto_faults_ = p.as_bool();
    } else if (n == "faults.failed_oru") {
      failed_oru_ = static_cast<int>(p.as_int());
      params_dirty_ = true;
    } else if (n == "faults.faulted_bcdu") {
      faulted_bcdu_ = static_cast<int>(p.as_int());
      params_dirty_ = true;
    } else if (n == "faults.faulted_ddcu") {
      faulted_ddcu_ = static_cast<int>(p.as_int());
      params_dirty_ = true;
    } else if (n == "faults.faulted_mbsu") {
      faulted_mbsu_ = static_cast<int>(p.as_int());
      params_dirty_ = true;
    } else if (n == "faults.offline_channel") {
      offline_channel_ = static_cast<int>(p.as_int());
      params_dirty_ = true;
    } else if (EpsParameterBridge::is_static_parameter(n)) {
      RCLCPP_INFO(get_logger(), "Static parameter '%s' changed; reconfigure to apply",
                  n.c_str());
    } else if (EpsParameterBridge::is_physics_parameter(n)) {
      params_dirty_ = true;
    }
  }
  return result;
}

void EpsNode::apply_fault_hooks()
{
  // A unit is faulted if either the what-if parameter or an injected fault
  // from the simulation names it.
  const auto failed_orus = injection_.indices("battery_oru_failed");
  const auto faulted_bcdus = injection_.indices("bcdu_fault");
  const auto lost_channels = injection_.indices("mbsu_channel_loss");
  const auto faulted_ddcus = injection_.indices("ddcu_fault");
  const auto faulted_mbsus = injection_.indices("mbsu_fault");
  for (int i = 0; i < eps_->n_orus(); ++i) {
    eps_->set_oru_failed(i, i == failed_oru_ || failed_orus.count(i) > 0);
  }
  for (int c = 0; c < eps_->n_channels(); ++c) {
    eps_->set_bcdu_fault(c, c == faulted_bcdu_ || faulted_bcdus.count(c) > 0);
    eps_->set_channel_offline(c, c == offline_channel_ || lost_channels.count(c) > 0);
  }
  for (int d = 0; d < eps_->n_ddcu(); ++d) {
    eps_->set_ddcu_fault(d, d == faulted_ddcu_ || faulted_ddcus.count(d) > 0);
  }
  for (int m = 0; m < eps_->n_mbsu(); ++m) {
    eps_->set_mbsu_fault(m, m == faulted_mbsu_ || faulted_mbsus.count(m) > 0);
  }
}

void EpsNode::refresh_physics_parameters()
{
  system::EpsParameters updated = bridge_->read_parameters();
  const auto & current = eps_->parameters();
  updated.n_channels = current.n_channels;
  updated.orus_per_channel = current.orus_per_channel;
  updated.n_mbsu = current.n_mbsu;
  updated.channel_mbsu = current.channel_mbsu;
  updated.ddcu_mbsu = current.ddcu_mbsu;
  updated.solar.array.degradation_factor *= injection_.array_factor();
  updated.battery.state_of_health *= injection_.health_factor();
  updated.demand_scale *= injection_.demand_scale();
  eps_->set_parameters(updated);
  apply_fault_hooks();
}

void EpsNode::on_fault_event(const FaultEvent::SharedPtr msg)
{
  // Only faults the simulation injects for EPS; EPS's own detected faults and
  // other subsystems' events are ignored.
  std::string params_json;
  double duration_s = 0.0;
  if (!eps_ || msg->subsystem_name != "eps" ||
    !EpsFaultInjection::parse_description(msg->description, params_json, duration_s))
  {
    return;
  }
  const auto params = EpsFaultInjection::parse_params(params_json);

  const std::string idx = EpsFaultInjection::index_param(msg->fault_type);
  if (!idx.empty()) {
    const auto it = params.find(idx);
    const int index = it != params.end() ? static_cast<int>(std::lround(it->second)) : 0;
    const int count = idx == "oru" ? eps_->n_orus() : idx == "channel" ? eps_->n_channels() :
      idx == "mbsu" ? eps_->n_mbsu() : eps_->n_ddcu();
    if (index < 0 || index >= count) {
      RCLCPP_WARN(get_logger(), "Ignoring injected %s: %s %d does not exist",
                  msg->fault_type.c_str(), idx.c_str(), index);
      return;
    }
  }

  std::string message;
  if (injection_.apply(msg->fault_type, params, duration_s, this->now().seconds(), message)) {
    RCLCPP_WARN(get_logger(), "Simulation fault: %s", message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Ignoring injected fault: %s", message.c_str());
  }
}

void EpsNode::on_world_state(const WorldState::SharedPtr msg)
{
  solar_flux_w_m2_ = msg->solar_flux_w_m2;
  in_eclipse_ = msg->in_eclipse;
}

void EpsNode::step()
{
  if (!eps_) {
    return;
  }
  const rclcpp::Time now = this->now();

  // Values set through the parameter callback are committed after it returns,
  // so live physics edits are picked up on the following step. Injected
  // faults that started, expired or were cleared are applied the same way.
  injection_.expire(now.seconds());
  if (params_dirty_ || injection_.revision() != applied_injection_revision_) {
    refresh_physics_parameters();
    if (const auto soc = injection_.take_pending_soc()) {
      eps_->set_battery_soc(*soc);
    }
    params_dirty_ = false;
    applied_injection_revision_ = injection_.revision();
  }

  double dt = 0.0;
  if (!first_step_) {
    const double measured = (now - last_step_time_).seconds();
    if (measured > 0.0 && measured < kMaxStepS) {
      dt = measured;
    }
  }
  first_step_ = false;
  last_step_time_ = now;

  system::EpsInput input{};
  input.solar_flux_w_m2 = solar_flux_w_m2_;
  input.in_eclipse = in_eclipse_;
  last_state_ = eps_->step(dt, input);

  publish_telemetry(now);
  publish_faults(now);
  heartbeat_pub_->publish(EpsDiagnostics::make_heartbeat(
      now, "eps", SubsystemHeartbeat::LIFECYCLE_ACTIVE, last_state_.healthy,
      last_state_.status_message));
}

void EpsNode::publish_telemetry(const rclcpp::Time & now)
{
  const auto & s = last_state_;
  using diagnostic_msgs::msg::DiagnosticStatus;

  auto publish_float = [](const Pub<Float64> & pub, double value) {
      Float64 msg;
      msg.data = value;
      pub->publish(msg);
    };
  publish_float(soc_pub_, s.battery_soc);
  publish_float(power_balance_pub_, s.power_balance_w);
  publish_float(array_power_pub_, s.array.array_power_w);
  publish_float(ssu_voltage_pub_, s.ssu.bus_voltage_v);
  publish_float(ssu_power_pub_, s.ssu.available_power_w);
  publish_float(ssu_current_pub_, s.ssu.available_current_a);
  publish_float(panel_temp_pub_, s.array.panel_temperature_c);
  publish_float(beta_pub_, s.array.beta_deg);

  int charging = 0;
  int discharging = 0;
  int idle = 0;
  int faulted = 0;
  int degraded = 0;
  double current_draw_a = 0.0;
  double bus_voltage_v = 0.0;
  for (std::size_t c = 0; c < s.channels.size(); ++c) {
    const auto & ch = s.channels[c];
    channel_pubs_[c]->publish(EpsDiagnostics::make_bcdu_status(
        now, "bcdu_" + std::to_string(c), ch));
    switch (ch.mode) {
      case bcdu::BcduMode::CHARGING: ++charging; break;
      case bcdu::BcduMode::DISCHARGING: ++discharging; break;
      case bcdu::BcduMode::IDLE: ++idle; break;
      case bcdu::BcduMode::FAULT: ++faulted; break;
    }
    if (ch.degraded) {
      ++degraded;
    }
    current_draw_a += ch.current_draw_a;
    bus_voltage_v = std::max(bus_voltage_v, ch.bus_voltage_v);
  }

  // Station-level BCDU summary: the mode most channels are in, with the
  // summed bus-side current.
  bcdu::BcduState summary{};
  const int most = std::max({charging, discharging, idle});
  summary.mode = faulted == static_cast<int>(s.channels.size()) ? bcdu::BcduMode::FAULT :
    most == discharging && discharging > 0 ? bcdu::BcduMode::DISCHARGING :
    most == charging && charging > 0 ? bcdu::BcduMode::CHARGING : bcdu::BcduMode::IDLE;
  summary.bus_voltage_v = bus_voltage_v;
  summary.regulation_voltage_v = eps_->parameters().bcdu.regulation_voltage_v;
  summary.current_draw_a = current_draw_a;
  summary.faulted = faulted > 0;
  summary.degraded = degraded > 0;
  summary.status_message = faulted > 0 ?
    std::to_string(faulted) + " BCDU(s) faulted" :
    degraded > 0 ? std::to_string(degraded) + " channel(s) degraded" : "nominal";
  bcdu_pub_->publish(EpsDiagnostics::make_bcdu_status(now, "bcdu", summary));

  int failed_orus = 0;
  int overheated_orus = 0;
  double max_temp_c = -1.0e9;
  double sum_temp_c = 0.0;
  const auto & battery_params = eps_->parameters().battery;
  const double capacity_ah = battery_params.capacity_ah *
    std::clamp(battery_params.state_of_health, 0.0, 1.0);
  for (std::size_t i = 0; i < s.orus.size(); ++i) {
    const auto & oru = s.orus[i];
    oru_pubs_[i]->publish(EpsDiagnostics::make_battery_state(
        now, "oru_" + std::to_string(i), oru, battery_params, capacity_ah));
    if (oru.health == battery::BatteryHealth::FAILED) {
      ++failed_orus;
    } else if (oru.health == battery::BatteryHealth::OVERHEAT) {
      ++overheated_orus;
    }
    max_temp_c = std::max(max_temp_c, oru.temperature_c);
    sum_temp_c += oru.temperature_c;
  }

  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = now;

  DiagnosticStatus eps;
  eps.name = "eps";
  eps.hardware_id = "ssos_eps/eps";
  EpsDiagnostics::add_value(eps, "generation_w", s.generation_w, 1);
  EpsDiagnostics::add_value(eps, "available_power_w", s.available_power_w, 1);
  EpsDiagnostics::add_value(eps, "supply_capability_w", s.supply_capability_w, 1);
  EpsDiagnostics::add_value(eps, "primary_load_w", s.primary_load_w, 1);
  EpsDiagnostics::add_value(eps, "load_demand_w", s.load_demand_w, 1);
  EpsDiagnostics::add_value(eps, "load_served_w", s.load_served_w, 1);
  EpsDiagnostics::add_value(eps, "unserved_load_w", s.unserved_load_w, 1);
  EpsDiagnostics::add_value(eps, "shed_loads", std::to_string(s.shed_loads));
  EpsDiagnostics::add_value(eps, "distribution_loss_w", s.distribution_loss_w, 1);
  EpsDiagnostics::add_value(eps, "power_balance_w", s.power_balance_w, 1);
  EpsDiagnostics::add_value(eps, "battery_power_w", s.battery_power_w, 1);
  EpsDiagnostics::add_value(eps, "battery_charge_w", s.battery_charge_w, 1);
  EpsDiagnostics::add_value(eps, "battery_discharge_w", s.battery_discharge_w, 1);
  EpsDiagnostics::add_value(eps, "shunted_power_w", s.shunted_power_w, 1);
  EpsDiagnostics::add_value(eps, "conversion_loss_w", s.conversion_loss_w, 1);
  EpsDiagnostics::add_value(eps, "battery_soc", s.battery_soc, 4);
  EpsDiagnostics::add_value(eps, "min_oru_soc", s.min_oru_soc, 4);
  EpsDiagnostics::add_value(eps, "stored_energy_kwh", s.stored_energy_wh / 1000.0, 2);
  EpsDiagnostics::add_value(eps, "solar_flux_w_m2", solar_flux_w_m2_, 1);
  EpsDiagnostics::add_value(eps, "in_eclipse", in_eclipse_ ? "true" : "false");
  eps.level = s.load_unserved ? DiagnosticStatus::ERROR :
    s.healthy ? DiagnosticStatus::OK : DiagnosticStatus::WARN;
  eps.message = s.status_message;
  diag.status.push_back(eps);

  DiagnosticStatus solar;
  solar.name = "eps/solar";
  solar.hardware_id = "ssos_eps/solar";
  EpsDiagnostics::add_value(solar, "array_power_w", s.array.array_power_w, 1);
  EpsDiagnostics::add_value(solar, "illumination", s.array.illumination, 3);
  EpsDiagnostics::add_value(solar, "beta_deg", s.array.beta_deg, 2);
  EpsDiagnostics::add_value(solar, "beta_derate", s.array.beta_derate, 3);
  EpsDiagnostics::add_value(solar, "gimbal_locked", s.array.gimbal_locked ? "true" : "false");
  EpsDiagnostics::add_value(solar, "panel_temperature_c", s.array.panel_temperature_c, 2);
  EpsDiagnostics::add_value(solar, "ssu_voltage_v", s.ssu.bus_voltage_v, 2);
  EpsDiagnostics::add_value(solar, "ssu_power_w", s.ssu.available_power_w, 1);
  EpsDiagnostics::add_value(solar, "ssu_current_a", s.ssu.available_current_a, 2);
  EpsDiagnostics::add_value(solar, "orbit_count", std::to_string(s.array.orbit_count));
  EpsDiagnostics::add_value(solar, "degradation_factor",
                            eps_->parameters().solar.array.degradation_factor, 3);
  solar.level = s.array.gimbal_locked ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
  solar.message = !s.array.sunlit ? "eclipse" :
    s.array.gimbal_locked ? "gimbal lock" : "generating";
  diag.status.push_back(solar);

  DiagnosticStatus bcdu;
  bcdu.name = "eps/bcdu";
  bcdu.hardware_id = "ssos_eps/bcdu";
  EpsDiagnostics::add_value(bcdu, "charging", std::to_string(charging));
  EpsDiagnostics::add_value(bcdu, "discharging", std::to_string(discharging));
  EpsDiagnostics::add_value(bcdu, "idle", std::to_string(idle));
  EpsDiagnostics::add_value(bcdu, "fault", std::to_string(faulted));
  EpsDiagnostics::add_value(bcdu, "degraded", std::to_string(degraded));
  EpsDiagnostics::add_value(bcdu, "current_draw_a", current_draw_a, 2);
  bcdu.level = faulted > 0 ? DiagnosticStatus::ERROR :
    degraded > 0 ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
  bcdu.message = summary.status_message;
  diag.status.push_back(bcdu);

  DiagnosticStatus battery;
  battery.name = "eps/battery";
  battery.hardware_id = "ssos_eps/battery";
  EpsDiagnostics::add_value(battery, "soc", s.battery_soc, 4);
  EpsDiagnostics::add_value(battery, "min_soc", s.min_oru_soc, 4);
  EpsDiagnostics::add_value(battery, "power_w", s.battery_power_w, 1);
  EpsDiagnostics::add_value(battery, "max_temperature_c", max_temp_c, 2);
  EpsDiagnostics::add_value(battery, "mean_temperature_c",
                            s.orus.empty() ? 0.0 : sum_temp_c / s.orus.size(), 2);
  EpsDiagnostics::add_value(battery, "failed_orus", std::to_string(failed_orus));
  EpsDiagnostics::add_value(battery, "overheated_orus", std::to_string(overheated_orus));
  battery.level = overheated_orus > 0 ? DiagnosticStatus::ERROR :
    failed_orus > 0 || s.low_soc ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
  battery.message = overheated_orus > 0 ?
    std::to_string(overheated_orus) + " ORU(s) over temperature" :
    failed_orus > 0 ? std::to_string(failed_orus) + " ORU(s) failed" :
    s.low_soc ? "state of charge below threshold" : "nominal";
  diag.status.push_back(battery);

  int mbsu_unstable = 0;
  int mbsu_single_source = 0;
  DiagnosticStatus mbsu;
  mbsu.name = "eps/mbsu";
  mbsu.hardware_id = "ssos_eps/mbsu";
  for (std::size_t m = 0; m < s.mbsus.size(); ++m) {
    const auto & st = s.mbsus[m];
    mbsu_pubs_[m]->publish(EpsDiagnostics::make_mbsu_status(
        now, "mbsu_" + std::to_string(m), st));
    EpsDiagnostics::add_value(
      mbsu, "mbsu_" + std::to_string(m),
      std::to_string(st.available_sources) + "/" + std::to_string(st.source_channels.size()) +
      " sources, " + std::to_string(static_cast<int>(st.supplied_w)) + " W");
    mbsu_unstable += st.unstable ? 1 : 0;
    mbsu_single_source += !st.unstable && st.redundancy_lost ? 1 : 0;
  }
  mbsu.level = mbsu_unstable > 0 ? DiagnosticStatus::ERROR :
    mbsu_single_source > 0 ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
  mbsu.message = mbsu_unstable > 0 ? std::to_string(mbsu_unstable) + " MBSU(s) unstable" :
    mbsu_single_source > 0 ? std::to_string(mbsu_single_source) + " MBSU(s) on one source" :
    "nominal";
  diag.status.push_back(mbsu);

  std::vector<int> active(s.ddcus.size(), 0);
  std::vector<int> shed(s.ddcus.size(), 0);
  DiagnosticStatus loads;
  loads.name = "eps/loads";
  loads.hardware_id = "ssos_eps/loads";
  for (const auto & l : s.loads) {
    if (l.ddcu >= 0 && static_cast<std::size_t>(l.ddcu) < s.ddcus.size()) {
      (l.shed ? shed : active)[static_cast<std::size_t>(l.ddcu)]++;
    }
    std::ostringstream value;
    value << std::fixed << std::setprecision(1) << l.served_w << "/" << l.requested_w <<
      " W, p" << l.priority << ", ddcu " << l.ddcu << (l.shed ? ", shed" : "");
    EpsDiagnostics::add_value(loads, "load:" + l.id, value.str());
  }
  loads.level = s.load_shed ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
  loads.message = s.load_shed ? std::to_string(s.shed_loads) + " load(s) shed" :
    std::to_string(s.loads.size()) + " loads supplied";
  diag.status.push_back(loads);

  int ddcu_faulted = 0;
  int ddcu_overloaded = 0;
  DiagnosticStatus ddcu;
  ddcu.name = "eps/ddcu";
  ddcu.hardware_id = "ssos_eps/ddcu";
  const auto & params = eps_->parameters();
  for (std::size_t d = 0; d < s.ddcus.size(); ++d) {
    const auto & st = s.ddcus[d];
    ddcu_pubs_[d]->publish(EpsDiagnostics::make_ddcu_status(
        now, "ddcu_" + std::to_string(d), st, params.ddcu_mbsu[d], params.ddcu.rated_power_w,
        active[d], shed[d]));
    EpsDiagnostics::add_value(
      ddcu, "ddcu_" + std::to_string(d),
      ddcu::to_string(st.mode) + ", " + std::to_string(static_cast<int>(st.output_power_w)) +
      " W, " + std::to_string(static_cast<int>(st.temperature_c)) + " C");
    ddcu_faulted += st.mode == ddcu::DdcuMode::TRIPPED ? 1 : 0;
    ddcu_overloaded += st.mode == ddcu::DdcuMode::OVERLOAD ? 1 : 0;
  }
  EpsDiagnostics::add_value(ddcu, "loss_w", s.distribution_loss_w, 1);
  ddcu.level = ddcu_faulted > 0 || s.ddcu_fault ? DiagnosticStatus::ERROR :
    ddcu_overloaded > 0 ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
  ddcu.message = ddcu_faulted > 0 ? std::to_string(ddcu_faulted) + " DDCU(s) tripped" :
    s.ddcu_fault ? "DDCU without primary power" :
    ddcu_overloaded > 0 ? std::to_string(ddcu_overloaded) + " DDCU(s) overloaded" : "nominal";
  diag.status.push_back(ddcu);

  DiagnosticStatus faults;
  faults.name = "eps/faults";
  faults.hardware_id = "ssos_eps/faults";
  for (const auto & f : injection_.active()) {
    EpsDiagnostics::add_value(
      faults, f.key, f.expires_s < 0.0 ? std::string("until cleared") :
      std::to_string(static_cast<int>(std::max(f.expires_s - now.seconds(), 0.0))) +
      " s left");
  }
  faults.level = injection_.active().empty() ? DiagnosticStatus::OK : DiagnosticStatus::WARN;
  faults.message = injection_.active().empty() ? "no injected faults" :
    std::to_string(injection_.active().size()) + " injected fault(s)";
  diag.status.push_back(faults);

  telemetry_pub_->publish(diag);
}

void EpsNode::publish_faults(const rclcpp::Time & now)
{
  const auto & s = last_state_;
  auto edge = [&](bool active, bool & reported, const std::string & type, uint8_t severity,
      const std::string & description, const std::vector<std::string> & affected) {
      if (active && !reported && enable_auto_faults_) {
        fault_pub_->publish(EpsDiagnostics::make_fault(
            now, "eps", type, severity, description, affected));
        reported = true;
      } else if (!active) {
        reported = false;
      }
    };

  edge(s.low_soc, low_soc_reported_, "low_battery_soc", FaultEvent::SEVERITY_CRITICAL,
       "EPS battery state of charge is below the configured threshold",
       {"/ssos/eps/battery/soc"});
  edge(s.load_unserved, load_unserved_reported_, "load_unserved",
       FaultEvent::SEVERITY_CRITICAL,
       "EPS cannot supply the full station load",
       {"/ssos/eps/diagnostics"});
  edge(s.bcdu_fault, bcdu_fault_reported_, "bcdu_fault", FaultEvent::SEVERITY_CRITICAL,
       "A BCDU has isolated its batteries",
       {"/ssos/eps/bcdu/status"});
  edge(s.oru_failed, oru_failed_reported_, "battery_oru_failed",
       FaultEvent::SEVERITY_WARNING,
       "A battery ORU has failed and is isolated",
       {"/ssos/eps/diagnostics"});
  edge(s.oru_overheat, oru_overheat_reported_, "battery_oru_overtemp",
       FaultEvent::SEVERITY_CRITICAL,
       "A battery ORU is above its temperature limit",
       {"/ssos/eps/diagnostics"});
  edge(s.ddcu_fault, ddcu_fault_reported_, "ddcu_fault", FaultEvent::SEVERITY_CRITICAL,
       "A DDCU carrying loads is tripped or without primary power",
       {"/ssos/eps/diagnostics"});
  edge(s.ddcu_overload, ddcu_overload_reported_, "ddcu_overload", FaultEvent::SEVERITY_WARNING,
       "Requested load on a DDCU exceeds its rating",
       {"/ssos/eps/diagnostics"});
  edge(s.mbsu_unstable, mbsu_unstable_reported_, "mbsu_bus_unstable",
       FaultEvent::SEVERITY_CRITICAL,
       "An MBSU has no usable source channel",
       {"/ssos/eps/diagnostics"});
  edge(s.redundancy_lost, redundancy_reported_, "mbsu_redundancy_lost",
       FaultEvent::SEVERITY_WARNING,
       "An MBSU is running on a single source channel",
       {"/ssos/eps/diagnostics"});
  edge(s.array_degraded, array_degraded_reported_, "solar_array_degraded",
       FaultEvent::SEVERITY_WARNING,
       "Solar array output is below beginning of life",
       {"/ssos/eps/solar/array_power_w"});
  edge(s.battery_degraded, battery_degraded_reported_, "battery_degraded",
       FaultEvent::SEVERITY_WARNING,
       "Battery state of health is below the minimum",
       {"/ssos/eps/battery/soc"});
}

void EpsNode::register_with_manager()
{
  if (!register_client_->wait_for_service(std::chrono::milliseconds(200))) {
    RCLCPP_WARN(get_logger(), "system_manager unavailable; continuing");
    return;
  }
  auto req = std::make_shared<RegisterSubsystem::Request>();
  req->subsystem_name = "eps";
  req->published_topics = {
    "/ssos/eps/diagnostics", "/ssos/eps/bcdu/status", "/ssos/eps/battery/soc",
    "/ssos/eps/power_balance_w", "/ssos/eps/solar/array_power_w",
    "/ssos/eps/solar/ssu_voltage_v", "/ssos/eps/mbsu/mbsu_<n>/status",
    "/ssos/eps/ddcu/ddcu_<n>/status"};
  req->subscribed_topics = {"/sim/world_state"};
  req->heartbeat_topic = "/ssos/eps/heartbeat";
  register_client_->async_send_request(req);
}

void EpsNode::handle_load_request(const Load::Request::SharedPtr req,
                                  Load::Response::SharedPtr res)
{
  res->ddcu_index = -1;
  res->granted_power_w = 0.0;
  res->supply_voltage = 0.0;
  if (!eps_) {
    res->success = false;
    res->message = "EPS is not configured";
    return;
  }

  system::LoadResult r{};
  if (req->action == Load::Request::ACTION_RELEASE) {
    r = eps_->release_load(req->load_id);
  } else if (req->action == Load::Request::ACTION_REQUEST) {
    r = eps_->request_load(req->load_id, req->power_w, req->priority, req->ddcu_index);
  } else {
    r.message = "unknown action " + std::to_string(req->action);
  }

  res->success = r.accepted;
  res->message = r.message;
  res->ddcu_index = r.accepted ? r.ddcu : -1;
  res->granted_power_w = r.granted_w;
  res->supply_voltage = r.supply_voltage_v;
  if (r.accepted) {
    RCLCPP_INFO(get_logger(), "Load request: %s", r.message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Load request '%s' rejected: %s", req->load_id.c_str(),
                r.message.c_str());
  }
}

}  // namespace nodes
}  // namespace ssos_eps
