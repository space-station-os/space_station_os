#include "ssos_sim/simulation_controller.hpp"

#include <cmath>
#include <fstream>
#include <functional>
#include <chrono>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace ssos_sim
{

// Forward declaration from default_world_model.cpp
std::unique_ptr<WorldModel> create_default_world_model();

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

SimulationController::SimulationController(const rclcpp::NodeOptions & options)
: LifecycleNode("simulation_controller", options)
{
  // Scenario parameters
  this->declare_parameter("scenario_file", "");
  this->declare_parameter("sim_rate_hz", 10.0);
  this->declare_parameter("sim_duration_s", 300.0);
  // Wall-time acceleration: sim time advances time_scale x faster than real time
  // (1.0 = real time). Read live each step so it can be changed at runtime, e.g.
  //   ros2 param set /simulation_controller time_scale 30.0
  this->declare_parameter("time_scale", 1.0);

  // Initial condition parameters (defaults = ISS-like LEO)
  this->declare_parameter("ic.altitude_km", 408.0);
  this->declare_parameter("ic.orbital_velocity_mps", 7660.0);
  this->declare_parameter("ic.atmospheric_o2_pct", 20.9);
  this->declare_parameter("ic.atmospheric_co2_ppm", 400.0);
  this->declare_parameter("ic.cabin_pressure_kpa", 101.3);
  this->declare_parameter("ic.cabin_temp_celsius", 22.0);
  this->declare_parameter("ic.solar_flux_w_m2", 1361.0);
  this->declare_parameter("ic.in_eclipse", false);
  // Starting position in the orbit as a fraction of the period [0, 1); the
  // default model is sunlit up to 0.65 and in eclipse after.
  this->declare_parameter("ic.orbit_phase", 0.0);

  RCLCPP_INFO(get_logger(), "SimulationController created (unconfigured)");
}

CallbackReturn SimulationController::on_configure(const rclcpp_lifecycle::State &)
{
  drill_.reset();
  RCLCPP_INFO(get_logger(), "Configuring...");

  sim_rate_hz_ = this->get_parameter("sim_rate_hz").as_double();
  sim_duration_s_ = this->get_parameter("sim_duration_s").as_double();

  // Build initial conditions from parameters
  WorldState ic;
  ic.altitude_km = this->get_parameter("ic.altitude_km").as_double();
  ic.orbital_velocity_mps = this->get_parameter("ic.orbital_velocity_mps").as_double();
  ic.atmospheric_o2_pct = this->get_parameter("ic.atmospheric_o2_pct").as_double();
  ic.atmospheric_co2_ppm = this->get_parameter("ic.atmospheric_co2_ppm").as_double();
  ic.cabin_pressure_kpa = this->get_parameter("ic.cabin_pressure_kpa").as_double();
  ic.cabin_temp_celsius = this->get_parameter("ic.cabin_temp_celsius").as_double();
  ic.solar_flux_w_m2 = this->get_parameter("ic.solar_flux_w_m2").as_double();
  ic.in_eclipse = this->get_parameter("ic.in_eclipse").as_bool();
  double orbit_phase = this->get_parameter("ic.orbit_phase").as_double();

  fault_schedule_.clear();
  scenario_name_.clear();
  const std::string scenario_file = this->get_parameter("scenario_file").as_string();
  if (!scenario_file.empty() && !load_scenario(scenario_file, ic, orbit_phase)) {
    return CallbackReturn::FAILURE;
  }

  // Create and initialize world model
  world_model_ = create_default_world_model();
  world_model_->initialize(ic);
  if (orbit_phase > 0.0) {
    world_model_->apply_perturbation("orbit_phase", orbit_phase);
  }

  // Create publishers
  world_state_pub_ = this->create_publisher<WorldState>("/sim/world_state", 10);
  clock_pub_ = this->create_publisher<Clock>("/clock", 10);
  fault_event_pub_ = this->create_publisher<FaultEvent>("/ssos/fault_event", 10);

  // Fault injection service
  inject_fault_srv_ = this->create_service<InjectFault>(
    "/sim/inject_fault",
    std::bind(&SimulationController::handle_inject_fault, this, _1, _2));

  sim_time_s_ = 0.0;

  RCLCPP_INFO(get_logger(), "Configured. Rate=%.1fHz, duration=%.0fs, altitude=%.1fkm",
    sim_rate_hz_, sim_duration_s_, ic.altitude_km);

  return CallbackReturn::SUCCESS;
}

CallbackReturn SimulationController::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating — simulation starting");
  world_state_pub_->on_activate();
  clock_pub_->on_activate();
  fault_event_pub_->on_activate();
  // A fresh run has no drill: tell every subsystem to drop any it still holds
  // from before this controller (re)started.
  for (const auto & [target, types] : drill_catalog()) {
    (void)types;
    publish_fault(target, kClearFaults, fault_description(
        "Injected fault", kClearFaults, "{}", 0.0));
  }

  // Start stepping now that publishers are active
  double period_ms = 1000.0 / sim_rate_hz_;
  step_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(period_ms)),
    std::bind(&SimulationController::step, this));

  return CallbackReturn::SUCCESS;
}

CallbackReturn SimulationController::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating — simulation paused at T+%.1fs", sim_time_s_);
  step_timer_->cancel();
  step_timer_.reset();
  world_state_pub_->on_deactivate();
  clock_pub_->on_deactivate();
  fault_event_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SimulationController::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up...");
  world_state_pub_.reset();
  clock_pub_.reset();
  fault_event_pub_.reset();
  inject_fault_srv_.reset();
  step_timer_.reset();
  world_model_.reset();
  fault_schedule_.clear();
  sim_time_s_ = 0.0;
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Step function — called at sim rate
// ---------------------------------------------------------------------------
void SimulationController::step()
{
  // Check if simulation is complete
  if (sim_time_s_ >= sim_duration_s_) {
    RCLCPP_INFO_ONCE(get_logger(), "Simulation complete at T+%.1fs", sim_time_s_);
    return;
  }

  // Sim time advances time_scale x faster than wall time. The step timer fires
  // at sim_rate_hz (wall), so each step advances time_scale / sim_rate_hz of sim
  // time. time_scale is read live so it can be changed at runtime.
  // time_scale <= 0 PAUSES the simulation: sim time and the world model stop
  // advancing, but /clock and world state keep publishing (at the frozen value)
  // so subscribers stay alive. This lets an external agent halt the sim, act,
  // and resume by restoring a positive time_scale.
  const double time_scale = this->get_parameter("time_scale").as_double();
  const bool paused = (time_scale <= 0.0);
  if (paused != paused_) {
    paused_ = paused;
    RCLCPP_INFO(get_logger(), "Simulation %s at T+%.1fs",
                paused ? "PAUSED" : "RESUMED", sim_time_s_);
  }
  const double dt = paused ? 0.0 : time_scale / sim_rate_hz_;

  // 1. Advance world model + fault schedule only while running.
  if (!paused) {
    world_model_->step(dt);
    sim_time_s_ += dt;
    if (drill_ && drill_->expires_s >= 0.0 && sim_time_s_ >= drill_->expires_s) {
      RCLCPP_INFO(get_logger(), "Drill %s on %s ended at T+%.1fs", drill_->type.c_str(),
                  drill_->target.c_str(), sim_time_s_);
      drill_.reset();
    }
    check_fault_schedule();
  }

  // 3. Publish world state (frozen when paused)
  auto ws = world_model_->get_state();
  ws.stamp = rclcpp::Clock().now();  // Use wall clock for stamp
  world_state_pub_->publish(ws);

  // 4. Publish sim clock
  auto clock_msg = Clock();
  clock_msg.clock.sec = static_cast<int32_t>(sim_time_s_);
  clock_msg.clock.nanosec = static_cast<uint32_t>(
    (sim_time_s_ - std::floor(sim_time_s_)) * 1e9);
  clock_pub_->publish(clock_msg);
}

// ---------------------------------------------------------------------------
// Fault injection
// ---------------------------------------------------------------------------
void SimulationController::handle_inject_fault(
  const InjectFault::Request::SharedPtr req,
  InjectFault::Response::SharedPtr res)
{
  std::string reason;
  if (!start_drill(req->target_subsystem, req->fault_type, req->parameters_json, req->duration_s,
    reason))
  {
    RCLCPP_WARN(get_logger(), "Fault injection rejected: %s", reason.c_str());
    res->success = false;
    res->message = reason;
    return;
  }
  RCLCPP_WARN(get_logger(), "Fault injection: target=%s type=%s duration=%.1fs",
    req->target_subsystem.c_str(), req->fault_type.c_str(), req->duration_s);
  publish_fault(req->target_subsystem, req->fault_type, fault_description(
      "Injected fault", req->fault_type, req->parameters_json, req->duration_s));
  res->success = true;
  res->message = "Fault injected: " + req->fault_type + " on " + req->target_subsystem;
}

bool SimulationController::start_drill(
  const std::string & target, const std::string & type, const std::string & parameters_json,
  double duration_s, std::string & reason)
{
  if (type == kClearFaults) {
    // Only the subsystem running the drill can end it.
    if (drill_ && drill_->target == target) {
      RCLCPP_INFO(get_logger(), "Drill %s on %s cleared", drill_->type.c_str(),
                  drill_->target.c_str());
      drill_.reset();
    }
    return true;
  }
  if (!validate_drill(target, type, parse_fault_params(parameters_json), reason)) {
    return false;
  }
  if (drill_) {
    reason = "drill " + drill_->type + " on " + drill_->target +
      " is still active; clear it before starting another";
    return false;
  }
  drill_ = ActiveDrill{target, type, duration_s > 0.0 ? sim_time_s_ + duration_s : -1.0};
  return true;
}

void SimulationController::publish_fault(
  const std::string & target, const std::string & type, const std::string & description,
  const std::string & interface)
{
  FaultEvent msg;
  msg.stamp = rclcpp::Clock().now();
  msg.subsystem_name = target;
  msg.fault_type = type;
  msg.severity = FaultEvent::SEVERITY_CRITICAL;
  msg.description = description;
  if (!interface.empty()) {
    msg.affected_interfaces = {interface};
  }
  fault_event_pub_->publish(msg);
}

void SimulationController::check_fault_schedule()
{
  for (auto & fault : fault_schedule_) {
    if (!fault.fired && sim_time_s_ >= fault.trigger_time_s) {
      fault.fired = true;
      fire_fault(fault);
    }
  }
}

void SimulationController::fire_fault(const ScheduledFault & fault)
{
  std::string reason;
  if (!start_drill(fault.target_subsystem, fault.fault_type, fault.parameters_json,
    fault.duration_s, reason))
  {
    RCLCPP_WARN(get_logger(), "Scheduled fault %s skipped: %s", fault.name.c_str(),
                reason.c_str());
    return;
  }
  RCLCPP_WARN(get_logger(), "Scheduled fault firing at T+%.1fs: %s on %s",
    sim_time_s_, fault.fault_type.c_str(), fault.target_subsystem.c_str());
  publish_fault(fault.target_subsystem, fault.fault_type, fault_description(
      "Scheduled fault", fault.name, fault.parameters_json, fault.duration_s),
    fault.target_interface);
}

// ---------------------------------------------------------------------------
// Scenario loading
// ---------------------------------------------------------------------------
std::string SimulationController::resolve_scenario_path(const std::string & scenario_file) const
{
  if (scenario_file.empty() || scenario_file.front() == '/') {
    return scenario_file;
  }
  {
    std::ifstream local(scenario_file);
    if (local.good()) {
      return scenario_file;
    }
  }
  std::string name = scenario_file;
  if (name.size() < 5 || name.substr(name.size() - 5) != ".yaml") {
    name += ".yaml";
  }
  try {
    return ament_index_cpp::get_package_share_directory("ssos_sim") + "/scenarios/" + name;
  } catch (const std::exception &) {
    return scenario_file;
  }
}

bool SimulationController::load_scenario(
  const std::string & scenario_file, WorldState & ic, double & orbit_phase)
{
  const std::string path = resolve_scenario_path(scenario_file);
  Scenario scenario;
  try {
    scenario = load_scenario_file(path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return false;
  }

  scenario_name_ = scenario.name;
  if (scenario.duration_s) {
    sim_duration_s_ = *scenario.duration_s;
  }
  if (scenario.sim_rate_hz && *scenario.sim_rate_hz > 0.0) {
    sim_rate_hz_ = *scenario.sim_rate_hz;
  }
  if (scenario.time_scale) {
    this->set_parameter(rclcpp::Parameter("time_scale", *scenario.time_scale));
  }

  const auto & v = scenario.initial_conditions;
  auto apply = [&v](const char * key, double & field) {
      const auto it = v.find(key);
      if (it != v.end()) {
        field = it->second;
      }
    };
  apply("altitude_km", ic.altitude_km);
  apply("orbital_velocity_mps", ic.orbital_velocity_mps);
  apply("atmospheric_o2_pct", ic.atmospheric_o2_pct);
  apply("atmospheric_co2_ppm", ic.atmospheric_co2_ppm);
  apply("cabin_pressure_kpa", ic.cabin_pressure_kpa);
  apply("cabin_temp_celsius", ic.cabin_temp_celsius);
  apply("solar_flux_w_m2", ic.solar_flux_w_m2);
  if (scenario.in_eclipse) {
    ic.in_eclipse = *scenario.in_eclipse;
  }
  if (scenario.orbit_phase) {
    orbit_phase = *scenario.orbit_phase;
  }
  fault_schedule_ = scenario.faults;

  RCLCPP_INFO(get_logger(), "Loaded scenario '%s' from %s: %zu scheduled fault(s)",
    scenario_name_.c_str(), path.c_str(), fault_schedule_.size());
  return true;
}

}  // namespace ssos_sim