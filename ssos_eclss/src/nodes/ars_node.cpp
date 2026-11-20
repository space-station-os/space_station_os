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
: rclcpp_lifecycle::LifecycleNode("ars_node", options),
  drills_(this, "ars"),
  health_(this, "ars",
    {"/ssos/ars/co2_removal_kg_day", "/ssos/ars/diagnostics", "/ssos/ars/bed_states",
      "/ssos/ars/cycle_phase"},
    {"/ssos/cabin/atmosphere"})
{
  // use_sim_time defaults declared by the node; allow override.
  if (!this->has_parameter("step_rate_hz")) {
    this->declare_parameter("step_rate_hz", step_rate_hz_);
  }
  if (!this->has_parameter("min_capture_efficiency")) {
    this->declare_parameter("min_capture_efficiency", min_capture_efficiency_);
  }
  if (!this->has_parameter("capture_time_constant_s")) {
    this->declare_parameter("capture_time_constant_s", capture_time_constant_s_);
  }
  if (!this->has_parameter("warm_start_s")) {
    this->declare_parameter("warm_start_s", warm_start_s_);
  }
  if (!this->has_parameter("enable_auto_faults")) {
    this->declare_parameter("enable_auto_faults", enable_auto_faults_);
  }
  if (!this->has_parameter("eps_coupling")) {
    this->declare_parameter("eps_coupling", eps_coupling_);
  }
  autostart_timer_ = EclssDiagnostics::maybe_autostart(this);
}

CallbackReturn ArsNode::on_configure(const rclcpp_lifecycle::State &)
{
  step_rate_hz_ = this->get_parameter("step_rate_hz").as_double();
  min_capture_efficiency_ = this->get_parameter("min_capture_efficiency").as_double();
  capture_time_constant_s_ = this->get_parameter("capture_time_constant_s").as_double();
  enable_auto_faults_ = this->get_parameter("enable_auto_faults").as_bool();
  warm_start_s_ = this->get_parameter("warm_start_s").as_double();

  bridge_ = std::make_unique<EclssParameterBridge>(this);
  bridge_->declare_ars_parameters(ars::default_ars_parameters());
  const ars::ArsParameters params = bridge_->read_ars_parameters();
  ars_ = std::make_unique<ars::FourBedSystem>(params);
  active_params_ = params;
  saturation_ = 0.0;

  // Design cabin conditions until cabin_node reports.
  cabin_.co2_partial_pressure_pa = units::torr_to_pa(params.operating.inlet_ppco2_torr);
  cabin_.h2o_partial_pressure_pa =
    gas::water_pp_from_rh(0.45, params.operating.cabin_temp_k);
  cabin_.temperature_k = params.operating.cabin_temp_k;
  cabin_.total_pressure_pa = params.operating.cabin_pressure_pa;

  // The station is already running: cycle the beds at design conditions so
  // they start loaded and mid-cycle rather than clean. At least one step, so
  // the first telemetry and power request are real values.
  constexpr double kWarmStepS = 5.0;
  double t = 0.0;
  do {
    last_result_ = ars_->step(kWarmStepS, cabin_);
    t += kWarmStepS;
  } while (t < warm_start_s_);
  full_power_w_ = last_result_.heater_power_w + last_result_.blower_power_w;
  design_flow_scfm_ = params.operating.inlet_flow_scfm;
  capture_ema_ = capture_efficiency();

  // Publishers.
  co2_removal_pub_ =
    this->create_publisher<std_msgs::msg::Float64>("/ssos/ars/co2_removal_kg_day", 10);
  telemetry_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/ssos/ars/diagnostics", 10);
  bed_states_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/ssos/ars/bed_states", 10);
  cycle_phase_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/ssos/ars/cycle_phase", 10);

  cabin_sub_ = this->create_subscription<space_station_interfaces::msg::CabinAtmosphere>(
    "/ssos/cabin/atmosphere", 10, std::bind(&ArsNode::on_cabin, this, _1));
  health_.configure();
  drills_.start();
  eps_coupling_ = this->get_parameter("eps_coupling").as_bool();
  power_.start(eps_coupling_);

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
  health_.activate();
  clock_.reset();
  const auto period = std::chrono::duration<double>(1.0 / std::max(step_rate_hz_, 1.0e-3));
  step_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ArsNode::step, this));

  RCLCPP_INFO(get_logger(), "ARS activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArsNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  co2_removal_pub_->on_deactivate();
  telemetry_pub_->on_deactivate();
  bed_states_pub_->on_deactivate();
  cycle_phase_pub_->on_deactivate();
  health_.deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArsNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  co2_removal_pub_.reset();
  telemetry_pub_.reset();
  bed_states_pub_.reset();
  cycle_phase_pub_.reset();
  cabin_sub_.reset();
  health_.cleanup();
  drills_.stop();
  power_.stop();
  ars_.reset();
  bridge_.reset();
  return CallbackReturn::SUCCESS;
}

void ArsNode::on_cabin(const space_station_interfaces::msg::CabinAtmosphere::SharedPtr msg)
{
  cabin_.total_pressure_pa = units::kpa_to_pa(msg->total_pressure_kpa);
  cabin_.temperature_k = units::celsius_to_kelvin(msg->temperature_c);
  cabin_.co2_partial_pressure_pa = units::kpa_to_pa(msg->co2_partial_pressure_kpa);
  cabin_.h2o_partial_pressure_pa = units::kpa_to_pa(msg->h2o_partial_pressure_kpa);
}

double ArsNode::capture_efficiency() const
{
  // CO2 drawn in with the process air [kg/s].
  const double co2_in = units::scfm_to_m3s(last_result_.blower_flow_scfm) *
    cabin_.co2_partial_pressure_pa / (units::R_GAS * cabin_.temperature_k) * units::M_CO2;
  return co2_in > 1.0e-12 ? last_result_.co2_removal_rate_kg_s / co2_in : 0.0;
}

void ArsNode::apply_parameters()
{
  // Drills act on the hardware the parameters describe: a degraded sorbent,
  // a stopped blower or dead bed heaters.
  active_params_ = bridge_->read_ars_parameters();
  if (drills_.active("ars_scrubber_degraded")) {
    active_params_.efficiency.capture_efficiency = drills_.param("capture_efficiency", 0.2);
  } else if (drills_.active("ars_blower_failure")) {
    active_params_.operating.inlet_flow_scfm = 0.01;
  } else if (drills_.active("ars_heater_failure")) {
    active_params_.heater.total_power_w = 0.0;
    active_params_.heater.central_power_w = 0.0;
  }
  apply_saturation();
}

void ArsNode::apply_saturation()
{
  // The system removal is the cycle-average capture; beds that are not being
  // regenerated capture less each half-cycle.
  ars::ArsParameters p = active_params_;
  p.efficiency.capture_efficiency *= 1.0 - 0.9 * saturation_;
  ars_->set_parameters(p);
}

void ArsNode::update_saturation(double dt)
{
  const double decay = std::exp(-dt / active_params_.cycle.half_cycle_s());
  const double before = saturation_;
  saturation_ = drills_.active("ars_heater_failure") ?
    1.0 - (1.0 - saturation_) * decay : saturation_ * decay;
  if (saturation_ < 1.0e-4) {
    saturation_ = 0.0;
  }
  if (saturation_ != before) {
    apply_saturation();
  }
}

void ArsNode::step()
{
  const rclcpp::Time now = this->now();
  if (drills_.update(now) || params_dirty_) {
    apply_parameters();
    params_dirty_ = false;
  }
  const double dt = clock_.advance(now);
  if (clock_.capped()) {
    static rclcpp::Clock wall(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(get_logger(), wall, 60000,
      "simulation is running faster than the ARS can follow; raise step_rate_hz");
  }

  // Cap the physics step so accelerated sim time (large dt) stays stable: split
  // dt into <=5 s chunks for the bed PDE (CFL-respecting) regardless of the
  // simulator's time_scale.
  constexpr double kMaxPhysicsDt = 5.0;
  const int chunks = std::max(1, static_cast<int>(std::ceil(dt / kMaxPhysicsDt)));
  const double h = dt / static_cast<double>(chunks);
  // Without the blower and bed heaters the 4BMS does nothing: hold the beds
  // and report no removal until EPS powers it again.
  const double supply = power_.powered_fraction();
  if (dt <= 0.0) {
    // Paused: hold the beds.
  } else if (supply > 0.999) {
    update_saturation(dt);
    for (int i = 0; i < chunks; ++i) {
      last_result_ = ars_->step(h, cabin_);
    }
    full_power_w_ = last_result_.heater_power_w + last_result_.blower_power_w;
    capture_ema_ += (capture_efficiency() - capture_ema_) *
      (1.0 - std::exp(-dt / std::max(capture_time_constant_s_, 1.0)));
  } else {
    last_result_.co2_removal_rate_kg_s = 0.0;
    last_result_.co2_removal_rate_kg_day = 0.0;
    last_result_.co2_desorbed_rate_kg_s = 0.0;
    last_result_.blower_flow_scfm = 0.0;
  }
  power_.update_demand(full_power_w_);

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
  kv("capture_efficiency", capture_efficiency());
  kv("capture_efficiency_avg", capture_ema_);
  kv("power_demand_w", full_power_w_);
  kv("power_supply_fraction", supply);
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

  std::string fault, msg = "nominal";
  if (!power_.powered()) {
    fault = "ars_power_loss";
    msg = power_.status();
  } else if (enable_auto_faults_ && last_result_.blower_flow_scfm < 0.1 * design_flow_scfm_) {
    fault = "ars_no_process_flow";
    msg = "no process air flow through the beds";
  } else if (enable_auto_faults_ && capture_ema_ < min_capture_efficiency_) {
    fault = "co2_removal_low";
    msg = "CO2 capture efficiency below limit";
  }
  health_.report(now, fault, msg, FaultEvent::SEVERITY_CRITICAL,
    {"/ssos/ars/co2_removal_kg_day"});
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
    const std::string & n = p.get_name();
    // Node-level (non-bridge) live params used directly in step().
    if (n == "enable_auto_faults") {
      enable_auto_faults_ = p.as_bool();
      continue;
    }
    if (n == "min_capture_efficiency") {
      min_capture_efficiency_ = p.as_double();
      continue;
    }
    if (n == "step_rate_hz") {
      step_rate_hz_ = p.as_double();
      continue;
    }
    if (EclssParameterBridge::is_static_parameter(n)) {
      RCLCPP_INFO(get_logger(),
                  "Static parameter '%s' changed; reconfigure to apply",
                  n.c_str());
    } else {
      params_dirty_ = true;
    }
  }
  return result;
}

}  // namespace nodes
}  // namespace ssos_eclss
