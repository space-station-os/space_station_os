#include "ssos_eclss/nodes/ars_node.hpp"

#include <algorithm>
#include <cmath>
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
  autostart_timer_ = EclssDiagnostics::maybe_autostart(this);
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
  bed_states_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/ssos/ars/bed_states", 10);
  cycle_phase_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/ssos/ars/cycle_phase", 10);
  heartbeat_pub_ =
    this->create_publisher<SubsystemHeartbeat>("/ssos/ars/heartbeat", 10);
  fault_pub_ = this->create_publisher<FaultEvent>("/ssos/fault_event", 10);

  // Subscriber to the simulation world state.
  world_state_sub_ = this->create_subscription<WorldState>(
    "/sim/world_state", 10, std::bind(&ArsNode::on_world_state, this, _1));

  // Closed-loop feedback: prefer the live cabin CO2 from cabin_node so the
  // ARS inlet tracks the actual cabin and the loop self-regulates.
  cabin_co2_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/ssos/cabin/co2_ppm", 10, std::bind(&ArsNode::on_cabin_co2, this, _1));

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
  bed_states_pub_->on_activate();
  cycle_phase_pub_->on_activate();
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
  bed_states_pub_->on_deactivate();
  cycle_phase_pub_->on_deactivate();
  heartbeat_pub_->on_deactivate();
  fault_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArsNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  co2_removal_pub_.reset();
  telemetry_pub_.reset();
  bed_states_pub_.reset();
  cycle_phase_pub_.reset();
  heartbeat_pub_.reset();
  fault_pub_.reset();
  world_state_sub_.reset();
  cabin_co2_sub_.reset();
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
  // Only seed CO2 from the simulator until the cabin_node feedback arrives;
  // afterwards on_cabin_co2() owns the inlet ppCO2 (closed loop).
  if (!have_cabin_co2_) {
    cabin_.co2_partial_pressure_pa =
      units::ppm_to_fraction(msg->atmospheric_co2_ppm) * total_p;
  }
  cabin_.h2o_partial_pressure_pa =
    gas::water_pp_from_rh(0.40, cabin_.temperature_k);
  have_world_state_ = true;
}

void ArsNode::on_cabin_co2(const std_msgs::msg::Float64::SharedPtr msg)
{
  // ppm -> partial pressure using the current cabin total pressure.
  cabin_.co2_partial_pressure_pa =
    units::ppm_to_fraction(msg->data) * cabin_.total_pressure_pa;
  have_cabin_co2_ = true;
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

  // Cap the physics step so accelerated sim time (large dt) stays stable: split
  // dt into <=5 s chunks for the bed PDE (CFL-respecting) regardless of the
  // simulator's time_scale.
  constexpr double kMaxPhysicsDt = 5.0;
  const int chunks = std::max(1, static_cast<int>(std::ceil(dt / kMaxPhysicsDt)));
  const double h = dt / static_cast<double>(chunks);
  for (int i = 0; i < chunks; ++i) {
    last_result_ = ars_->step(h, cabin_);
  }

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

  // ---- Per-bed states (drives the GUI bed bars) ----
  // /ssos/ars/bed_states  std_msgs/Float64MultiArray, 12 elements, bed order
  // [ADS train0, ADS train1, DES train0, DES train1]:
  //   [0..3]  loading fraction (0-1)
  //   [4..7]  solid temperature [K]
  //   [8..11] mode code (0=ADSORBING, 1=DESORBING, 2=AIR_SAVE, 3=VACUUM, 4=other)
  {
    const ars::CycleStateMachine & cyc = ars_->cycle();
    auto mode_code = [](ars::BedMode m) -> double {
      switch (m) {
        case ars::BedMode::ADSORBING: return 0.0;
        case ars::BedMode::DESORBING: return 1.0;
        case ars::BedMode::AIR_SAVE: return 2.0;
        case ars::BedMode::VACUUM: return 3.0;
        default: return 4.0;
      }
    };
    std_msgs::msg::Float64MultiArray beds;
    beds.data.resize(12);
    // Loading fractions.
    beds.data[0] = ars_->adsorbent_bed(0).loading_fraction();
    beds.data[1] = ars_->adsorbent_bed(1).loading_fraction();
    beds.data[2] = ars_->desiccant_bed(0).loading_fraction();
    beds.data[3] = ars_->desiccant_bed(1).loading_fraction();
    // Temperatures [K].
    beds.data[4] = ars_->adsorbent_bed(0).mean_solid_temperature();
    beds.data[5] = ars_->adsorbent_bed(1).mean_solid_temperature();
    beds.data[6] = ars_->desiccant_bed(0).mean_solid_temperature();
    beds.data[7] = ars_->desiccant_bed(1).mean_solid_temperature();
    // Mode codes per train (adsorbent + desiccant share the train's command).
    beds.data[8] = mode_code(cyc.command_for_train(0).adsorbent_mode);
    beds.data[9] = mode_code(cyc.command_for_train(1).adsorbent_mode);
    beds.data[10] = mode_code(cyc.command_for_train(0).desiccant_mode);
    beds.data[11] = mode_code(cyc.command_for_train(1).desiccant_mode);
    bed_states_pub_->publish(beds);

    // ---- Cycle phase (drives the GUI 10-60-10 cycle bar) ----
    // /ssos/ars/cycle_phase  std_msgs/Float64MultiArray, 3 elements:
    //   [0] elapsed time in the current half-cycle [s]
    //   [1] half-cycle duration [s]
    //   [2] index of the adsorbing train (0/1)
    std_msgs::msg::Float64MultiArray phase;
    phase.data = {cyc.time_in_half_cycle(), cyc.timing().half_cycle_s(),
                  static_cast<double>(cyc.adsorbing_train())};
    cycle_phase_pub_->publish(phase);
  }

  // Fault detection (edge-triggered: publish a fault only when health changes,
  // never every step, so a momentary dip can't spam the fault bus / flip state).
  bool healthy = true;
  std::string health_msg = "nominal";
  if (EclssDiagnostics::ars_co2_removal_low(last_result_.co2_removal_rate_kg_day,
                                            co2_required_kg_day_)) {
    healthy = false;
    health_msg = "CO2 removal below requirement";
  }
  if (!healthy && was_healthy_) {
    fault_pub_->publish(EclssDiagnostics::make_fault(
      now, "ars", "co2_removal_below_requirement",
      FaultEvent::SEVERITY_CRITICAL, health_msg, {"/ssos/ars/co2_removal_kg_day"}));
  }
  was_healthy_ = healthy;

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
  req->published_topics = {"/ssos/ars/co2_removal_kg_day", "/ssos/ars/diagnostics",
                           "/ssos/ars/bed_states", "/ssos/ars/cycle_phase"};
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
