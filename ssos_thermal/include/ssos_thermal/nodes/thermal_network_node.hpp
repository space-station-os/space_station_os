#ifndef SSOS_THERMAL__NODES__THERMAL_NETWORK_NODE_HPP_
#define SSOS_THERMAL__NODES__THERMAL_NETWORK_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "space_station_interfaces/action/coolant.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/msg/thermal_link_flows_array.hpp"
#include "space_station_interfaces/msg/thermal_node_data_array.hpp"
#include "space_station_interfaces/srv/register_subsystem.hpp"

#include "ssos_thermal/network/thermal_network.hpp"
#include "ssos_thermal/nodes/thermal_diagnostics.hpp"

// Lifecycle node wrapping the ROS-free ThermalNetwork solver. Integrates the
// node/link graph every tick, drives the coolant-loop action when the
// average temperature crosses cooling_trigger_threshold, and publishes
// telemetry alongside a heartbeat/fault pair on the ssos_core contract.

namespace ssos_thermal
{
namespace nodes
{

using RegisterSubsystem = space_station_interfaces::srv::RegisterSubsystem;
using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using FaultEvent = space_station_interfaces::msg::FaultEvent;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ThermalNetworkNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ThermalNetworkNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
  using GoalHandleCoolant =
    rclcpp_action::ClientGoalHandle<space_station_interfaces::action::Coolant>;

  void updateSimulation();
  void coolingCallback();
  void publishThermalNetworkDiag(const network::ThermalNetwork::Hottest & hottest);
  void registerWithManager();
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params);

  // Physics
  std::unique_ptr<network::ThermalNetwork> network_;
  ThermalDiagnostics diag_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<
    space_station_interfaces::msg::ThermalNodeDataArray>::SharedPtr node_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    space_station_interfaces::msg::ThermalLinkFlowsArray>::SharedPtr link_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
    diag_pub_;
  rclcpp_lifecycle::LifecyclePublisher<SubsystemHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp_lifecycle::LifecyclePublisher<FaultEvent>::SharedPtr fault_pub_;

  // Action client + service client
  rclcpp_action::Client<space_station_interfaces::action::Coolant>::SharedPtr cooling_client_;
  rclcpp::Client<RegisterSubsystem>::SharedPtr register_client_;

  // Timers
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;

  // Parameter callback handle
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Dynamic parameters
  bool enable_failure_ = false;
  bool enable_cooling_ = true;
  bool cooling_active_ = false;
  double cooling_rate_ = 0.05;
  double cooling_trigger_threshold_ = 330.0;
  double max_temp_threshold_ = 420.0;
  double thermal_update_dt_ = 0.5;
  std::string thermal_config_file_ = "config/thermal_nodes.yaml";

  double feedback_latest_temp_ = 0.0;
};

}  // namespace nodes
}  // namespace ssos_thermal

#endif  // SSOS_THERMAL__NODES__THERMAL_NETWORK_NODE_HPP_
