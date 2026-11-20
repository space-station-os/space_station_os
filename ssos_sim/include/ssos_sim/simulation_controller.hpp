#ifndef SSOS_SIM__SIMULATION_CONTROLLER_HPP_
#define SSOS_SIM__SIMULATION_CONTROLLER_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "space_station_interfaces/msg/world_state.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/srv/inject_fault.hpp"

#include "ssos_sim/scenario_loader.hpp"
#include "ssos_sim/world_model.hpp"

namespace ssos_sim
{

using WorldState = space_station_interfaces::msg::WorldState;
using FaultEvent = space_station_interfaces::msg::FaultEvent;
using InjectFault = space_station_interfaces::srv::InjectFault;
using Clock = rosgraph_msgs::msg::Clock;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Central simulation controller.
/// Owns the world clock, world model, scenario config, and fault schedule.
class SimulationController : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SimulationController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /// Get current sim time in seconds (for testing)
  double get_sim_time() const { return sim_time_s_; }

  /// Get world model (for testing)
  const WorldModel & get_world_model() const { return *world_model_; }

  /// Loaded scenario name and fault schedule (for testing)
  const std::string & scenario_name() const { return scenario_name_; }
  const std::vector<ScheduledFault> & fault_schedule() const { return fault_schedule_; }

  /// The drill currently running: one fault at a time across the station.
  struct ActiveDrill
  {
    std::string target;
    std::string type;
    double expires_s;   // < 0 = until cleared
  };
  const std::optional<ActiveDrill> & active_drill() const { return drill_; }

private:
  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<WorldState>::SharedPtr world_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<Clock>::SharedPtr clock_pub_;
  rclcpp_lifecycle::LifecyclePublisher<FaultEvent>::SharedPtr fault_event_pub_;

  // Services
  rclcpp::Service<InjectFault>::SharedPtr inject_fault_srv_;

  // Timer
  rclcpp::TimerBase::SharedPtr step_timer_;

  // World model
  std::unique_ptr<WorldModel> world_model_;

  // Scenario config
  double sim_rate_hz_{10.0};
  double sim_duration_s_{300.0};
  double sim_time_s_{0.0};
  bool paused_{false};  // true while time_scale <= 0 (sim time frozen)
  std::string scenario_name_;
  std::vector<ScheduledFault> fault_schedule_;
  std::optional<ActiveDrill> drill_;

  // Callbacks
  void step();
  void handle_inject_fault(
    const InjectFault::Request::SharedPtr req,
    InjectFault::Response::SharedPtr res);

  // Scenario loading: resolve the file, apply its settings over the
  // parameters and load its fault schedule. Returns false on error.
  bool load_scenario(const std::string & scenario_file, WorldState & ic, double & orbit_phase);
  std::string resolve_scenario_path(const std::string & scenario_file) const;

  // Fault processing
  void check_fault_schedule();
  void fire_fault(const ScheduledFault & fault);
  // Admit a drill (or a clear_faults) if no other drill is running.
  bool start_drill(const std::string & target, const std::string & type,
                   const std::string & parameters_json, double duration_s, std::string & reason);
  void publish_fault(const std::string & target, const std::string & type,
                     const std::string & description, const std::string & interface = "");
};

}  // namespace ssos_sim

#endif  // SSOS_SIM__SIMULATION_CONTROLLER_HPP_