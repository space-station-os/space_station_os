#ifndef SSOS_THERMAL__NODES__COOLANT_NODE_HPP_
#define SSOS_THERMAL__NODES__COOLANT_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "space_station_interfaces/action/coolant.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/srv/register_subsystem.hpp"
#include "space_station_interfaces/srv/vent_heat.hpp"

#include "ssos_thermal/coolant/coolant_loop.hpp"
#include "ssos_thermal/nodes/thermal_diagnostics.hpp"

// Lifecycle node wrapping the ROS-free CoolantLoop model. Serves the
// Coolant action (goal: a component's temperature to cool; feedback:
// internal/ammonia temperature + vented heat each physics step) that both
// thermal_network_node's cooling client and the GUI's ThermalWidget consume
// on /coolant_heat_transfer.
//
// Ported from space_station_thermal_control's CoolantActionServer, keeping
// only the part anything actually depends on: the action's cooldown loop.
// Left behind (see REFACTOR_PLAN.md): a Behavior-Tree tick loop whose
// condition variables (current_temp_, ammonia_heat_kj_) were never written
// to by anything (so it always evaluated the same static branch), two BT
// action clients (vent_client_, wrs_client_) that were declared but never
// constructed (so those leaves always failed), a recycleWater() method
// never called from anywhere, and two telemetry publishers
// (internal_loop_pub_, external_loop_pub_) that were declared but never
// published to.

namespace ssos_thermal
{
namespace nodes
{

using RegisterSubsystem = space_station_interfaces::srv::RegisterSubsystem;
using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using FaultEvent = space_station_interfaces::msg::FaultEvent;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CoolantNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CoolantNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
  using Coolant = space_station_interfaces::action::Coolant;
  using GoalHandleCoolant = rclcpp_action::ServerGoalHandle<Coolant>;

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Coolant::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleCoolant> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleCoolant> goal_handle);
  void execute(const std::shared_ptr<GoalHandleCoolant> goal_handle);
  void publishHeartbeat();
  void registerWithManager();

  // Physics
  std::unique_ptr<coolant::CoolantLoop> loop_;
  double target_temp_c_ = 25.0;

  // Action server
  rclcpp_action::Server<Coolant>::SharedPtr action_server_;

  // Best-effort radiator vent-heat client (space_station_thermal_control's
  // `radiator` node, not part of this package -- see REFACTOR_PLAN.md for
  // why only thermal_network + cooling_server are in scope here).
  rclcpp::Client<space_station_interfaces::srv::VentHeat>::SharedPtr radiator_client_;
  rclcpp::Client<RegisterSubsystem>::SharedPtr register_client_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<SubsystemHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp_lifecycle::LifecyclePublisher<FaultEvent>::SharedPtr fault_pub_;

  ThermalDiagnostics diag_;

  // Timers
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;

  // Dynamic parameters
  double mass_kg_ = 200.0;
  double specific_heat_j_per_kg_c_ = 4186.0;
  double heat_transfer_efficiency_ = 0.85;
  double vent_threshold_kj_ = 250.0;
};

}  // namespace nodes
}  // namespace ssos_thermal

#endif  // SSOS_THERMAL__NODES__COOLANT_NODE_HPP_
