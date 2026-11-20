#ifndef SSOS_CORE__SYSTEM_MANAGER_HPP_
#define SSOS_CORE__SYSTEM_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "space_station_interfaces/msg/system_state.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/srv/register_subsystem.hpp"

namespace ssos_core
{

using SystemState = space_station_interfaces::msg::SystemState;
using FaultEvent = space_station_interfaces::msg::FaultEvent;
using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using RegisterSubsystem = space_station_interfaces::srv::RegisterSubsystem;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Tracked state for a single registered subsystem
struct SubsystemRecord
{
  std::string name;
  std::vector<std::string> published_topics;
  std::vector<std::string> subscribed_topics;
  std::string heartbeat_topic;
  SubsystemHeartbeat last_heartbeat;
  bool has_active_fault{false};
  std::string active_fault_desc;
  rclcpp::Subscription<SubsystemHeartbeat>::SharedPtr heartbeat_sub;
  rclcpp::Time last_heard;
  bool registered{false};
  bool heard{false};              // at least one heartbeat since registration
  bool reported_healthy{false};   // health as the subsystem itself reports it
  rclcpp::Time healthy_since;      // start of the current healthy streak
  rclcpp::Time fault_time;         // when the active fault was raised
};

/// Central system manager node.
/// Aggregates subsystem heartbeats and fault events into a global system state.
class SystemManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SystemManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /// Get current system state (for testing)
  uint8_t get_current_state() const { return current_state_; }

  /// Evaluate and return what the system state should be (for testing)
  uint8_t evaluate_system_state() const;
  // Unhealthy, inactive, faulted or silent past the timeout. A subsystem that
  // has registered but not yet sent a heartbeat only counts once it times out.
  bool is_degraded(const SubsystemRecord & record, const rclcpp::Time & now) const;

private:
  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<SystemState>::SharedPtr state_pub_;

  // Subscribers
  rclcpp::Subscription<FaultEvent>::SharedPtr fault_sub_;

  // Services
  rclcpp::Service<RegisterSubsystem>::SharedPtr register_srv_;

  // Timer for periodic state evaluation
  rclcpp::TimerBase::SharedPtr eval_timer_;

  // State
  uint8_t current_state_{SystemState::INIT};
  std::map<std::string, SubsystemRecord> subsystems_;

  // Parameters
  double heartbeat_timeout_s_{5.0};
  double eval_period_s_{1.0};
  double recovery_hold_s_{10.0};  // healthy time before a fault clears (< 0 disables)

  // Callbacks
  void on_fault_event(const FaultEvent::SharedPtr msg);
  void on_heartbeat(const std::string & subsystem_name, const SubsystemHeartbeat::SharedPtr msg);
  void handle_register(
    const RegisterSubsystem::Request::SharedPtr req,
    RegisterSubsystem::Response::SharedPtr res);

  /// Core state evaluation logic
  void run_evaluation();

  /// Clear the faults of subsystems that have reported healthy for
  /// recovery_hold_s since the fault. Returns the names that recovered.
  std::vector<std::string> clear_recovered_faults();

  /// Publish a state transition
  void publish_state(uint8_t new_state, const std::string & reason);
};

}  // namespace ssos_core

#endif  // SSOS_CORE__SYSTEM_MANAGER_HPP_