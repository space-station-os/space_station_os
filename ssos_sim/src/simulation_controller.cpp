#include "ssos_sim/simulation_controller.hpp"

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

  RCLCPP_INFO(get_logger(), "SimulationController created (unconfigured)");
}

CallbackReturn SimulationController::on_configure(const rclcpp_lifecycle::State &)
{
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

  // Create and initialize world model
  world_model_ = create_default_world_model();
  world_model_->initialize(ic);

  // Create publishers
  world_state_pub_ = this->create_publisher<WorldState>("/sim/world_state", 10);
  clock_pub_ = this->create_publisher<Clock>("/clock", 10);
  fault_event_pub_ = this->create_publisher<FaultEvent>("/ssos/fault_event", 10);

  // Fault injection service
  inject_fault_srv_ = this->create_service<InjectFault>(
    "/sim/inject_fault",
    std::bind(&SimulationController::handle_inject_fault, this, _1, _2));

  sim_time_s_ = 0.0;
  fault_schedule_.clear();

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
  RCLCPP_WARN(get_logger(), "Fault injection: target=%s type=%s duration=%.1fs",
    req->target_subsystem.c_str(),
    req->fault_type.c_str(),
    req->duration_s);

  // Build and publish a FaultEvent for the system_manager
  auto fault_msg = FaultEvent();
  fault_msg.stamp = rclcpp::Clock().now();
  fault_msg.subsystem_name = req->target_subsystem;
  fault_msg.fault_type = req->fault_type;
  fault_msg.severity = FaultEvent::SEVERITY_CRITICAL;  // Default for injected faults
  fault_msg.description = "Injected fault: " + req->fault_type +
    " (params: " + req->parameters_json + ")";
  fault_msg.affected_interfaces = {};

  fault_event_pub_->publish(fault_msg);

  // Apply perturbation to world model if applicable
  if (req->fault_type == "sensor_stuck_at") {
    // Parse stuck value from JSON params (minimal parsing)
    // Expected format: {"stuck_value": 0.0}
    // For now, apply as O2 perturbation if target is eclss
    if (req->target_subsystem == "eclss") {
      // Simple JSON extraction for stuck_value
      auto pos = req->parameters_json.find("stuck_value");
      if (pos != std::string::npos) {
        auto colon = req->parameters_json.find(':', pos);
        if (colon != std::string::npos) {
          double val = std::stod(req->parameters_json.substr(colon + 1));
          world_model_->apply_perturbation("atmospheric_o2_pct", val);
          RCLCPP_WARN(get_logger(), "Applied O2 perturbation: %.1f%%", val);
        }
      }
    }
  }

  res->success = true;
  res->message = "Fault injected: " + req->fault_type + " on " + req->target_subsystem;
}

void SimulationController::check_fault_schedule()
{
  for (auto & fault : fault_schedule_) {
    if (!fault.fired && sim_time_s_ >= fault.trigger_time_s) {
      RCLCPP_WARN(get_logger(), "Scheduled fault firing at T+%.1fs: %s on %s",
        sim_time_s_, fault.fault_type.c_str(), fault.target_subsystem.c_str());
      fire_fault(fault);
      fault.fired = true;
    }
  }
}

void SimulationController::fire_fault(const ScheduledFault & fault)
{
  auto fault_msg = FaultEvent();
  fault_msg.stamp = rclcpp::Clock().now();
  fault_msg.subsystem_name = fault.target_subsystem;
  fault_msg.fault_type = fault.fault_type;
  fault_msg.severity = FaultEvent::SEVERITY_CRITICAL;
  fault_msg.description = "Scheduled fault: " + fault.name;
  if (!fault.target_interface.empty()) {
    fault_msg.affected_interfaces = {fault.target_interface};
  }

  fault_event_pub_->publish(fault_msg);
}

// ---------------------------------------------------------------------------
// Scenario loading (placeholder — YAML parsing comes with ssos_scenarios)
// ---------------------------------------------------------------------------
bool SimulationController::load_scenario(const std::string & scenario_file)
{
  (void)scenario_file;
  RCLCPP_INFO(get_logger(), "Scenario loading not yet implemented — using parameter-based config");
  return true;
}

WorldState SimulationController::parse_initial_conditions(const std::string & scenario_file)
{
  (void)scenario_file;
  return world_model_->get_state();
}

std::vector<ScheduledFault> SimulationController::parse_fault_schedule(
  const std::string & scenario_file)
{
  (void)scenario_file;
  return {};
}

}  // namespace ssos_sim