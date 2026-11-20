#ifndef SSOS_ECLSS__NODES__NODE_SUPPORT_HPP_
#define SSOS_ECLSS__NODES__NODE_SUPPORT_HPP_

#include <optional>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/float64.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/srv/register_subsystem.hpp"
#include "ssos_sim/fault_format.hpp"

// Pieces every ECLSS node shares: the simulation-time step, the drill
// injected through the simulation path, and the heartbeat with a fault event
// on each transition to unhealthy.

namespace ssos_eclss
{
namespace nodes
{

/// Simulation time since the previous tick of a wall-clock timer. Returns 0
/// on the first tick, while the simulation is paused and if time goes
/// backwards, so physics only advances with the simulation.
class StepClock
{
public:
  explicit StepClock(double max_step_s = 3600.0) : max_step_s_(max_step_s) {}

  void reset() {started_ = false;}
  double advance(const rclcpp::Time & now);
  /// True if the last advance dropped simulation time at the step cap.
  bool capped() const {return capped_;}

private:
  double max_step_s_;
  bool started_{false};
  bool capped_{false};
  rclcpp::Time last_;
};

/// kg/day on the topics, kg/s in the physics.
constexpr double kKgDayToKgS = 1.0 / 86400.0;

/// A value latched from a std_msgs/Float64 topic, scaled on arrival. If the
/// publisher goes quiet for longer than the timeout (wall time, so a paused
/// simulation does not count), it reads as the fallback, or as the last value
/// when there is no fallback.
class LatchedValue
{
public:
  LatchedValue(double scale, std::optional<double> fallback)
  : scale_(scale), fallback_(fallback) {}

  void subscribe(rclcpp_lifecycle::LifecycleNode * node, const std::string & topic);
  void reset();
  double get() const;
  bool fresh() const;

private:
  double scale_;
  std::optional<double> fallback_;
  double value_{0.0};
  bool heard_{false};
  rclcpp::Clock wall_{RCL_STEADY_TIME};
  rclcpp::Time stamp_{0, 0, RCL_STEADY_TIME};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
};

/// The drill a subsystem is running, taken from /ssos/fault_event events
/// addressed to it. Expiry runs on node (simulation) time.
class InjectedFaults
{
public:
  /// Accepts the drills ssos_sim::drill_catalog() lists for this subsystem.
  InjectedFaults(rclcpp_lifecycle::LifecycleNode * node, std::string subsystem);

  /// Subscribe (on configure); the drill state is kept for the node's life.
  void start();
  void stop();
  /// Expire the drill at now; true if the drill changed since the last call.
  bool update(const rclcpp::Time & now);

  const ssos_sim::FaultTracker & tracker() const {return tracker_;}
  bool active(const std::string & type) const {return tracker_.active(type);}
  double param(const std::string & name, double fallback) const
  {
    return tracker_.param(name, fallback);
  }

private:
  void on_fault(const space_station_interfaces::msg::FaultEvent::SharedPtr msg);

  rclcpp_lifecycle::LifecycleNode * node_;
  std::string subsystem_;
  rclcpp::Subscription<space_station_interfaces::msg::FaultEvent>::SharedPtr sub_;
  ssos_sim::FaultTracker tracker_;
  uint64_t seen_revision_{0};
};

/// Registration with system_manager, a heartbeat every step, and one
/// FaultEvent each time the subsystem turns unhealthy or the reason changes.
/// Registration is retried each step until system_manager answers.
class HealthReporter
{
public:
  HealthReporter(rclcpp_lifecycle::LifecycleNode * node, std::string subsystem,
                 std::vector<std::string> published, std::vector<std::string> subscribed);

  void configure();
  void activate();
  void deactivate();
  void cleanup();

  /// fault_type is empty when healthy.
  void report(const rclcpp::Time & now, const std::string & fault_type,
              const std::string & message, uint8_t severity,
              const std::vector<std::string> & interfaces = {});

private:
  void try_register();

  rclcpp_lifecycle::LifecycleNode * node_;
  std::string subsystem_;
  std::vector<std::string> published_;
  std::vector<std::string> subscribed_;
  rclcpp::Client<space_station_interfaces::srv::RegisterSubsystem>::SharedPtr register_client_;
  bool registered_{false};
  bool register_pending_{false};
  rclcpp::Clock wall_{RCL_STEADY_TIME};
  rclcpp::Time register_sent_{0, 0, RCL_STEADY_TIME};
  rclcpp_lifecycle::LifecyclePublisher<space_station_interfaces::msg::SubsystemHeartbeat>::
  SharedPtr heartbeat_pub_;
  rclcpp_lifecycle::LifecyclePublisher<space_station_interfaces::msg::FaultEvent>::SharedPtr
    fault_pub_;
  std::string last_fault_;
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__NODE_SUPPORT_HPP_
