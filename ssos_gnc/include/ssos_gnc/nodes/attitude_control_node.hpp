#ifndef SSOS_GNC__NODES__ATTITUDE_CONTROL_NODE_HPP_
#define SSOS_GNC__NODES__ATTITUDE_CONTROL_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/gnc_mode_state.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/msg/world_state.hpp"
#include "space_station_interfaces/srv/register_subsystem.hpp"
#include "space_station_interfaces/srv/set_actuation_mode.hpp"

#include "ssos_gnc/flight/attitude_control_system.hpp"
#include "ssos_gnc/nodes/gnc_diagnostics.hpp"
#include "ssos_gnc/nodes/gnc_parameter_bridge.hpp"
#include "ssos_gnc/nodes/thruster_geometry_loader.hpp"

namespace ssos_gnc
{

namespace nodes
{

using WorldState = space_station_interfaces::msg::WorldState;
using GncModeState = space_station_interfaces::msg::GncModeState;
using RegisterSubsystem = space_station_interfaces::srv::RegisterSubsystem;
using SetActuationMode = space_station_interfaces::srv::SetActuationMode;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AttitudeControlNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit AttitudeControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  const flight::ControlResult & last_result() const {return last_result_;}

private:
  void step();

  void on_attitude(const geometry_msgs::msg::Quaternion::SharedPtr msg);
  void on_rate(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void on_gimbal(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void on_pose_ref(const geometry_msgs::msg::Quaternion::SharedPtr msg);
  void on_world_state(const WorldState::SharedPtr msg);
  void on_unload_request(const std_msgs::msg::Bool::SharedPtr msg);

  void handle_set_mode(
    const SetActuationMode::Request::SharedPtr req,
    SetActuationMode::Response::SharedPtr res);

  void register_with_manager();
  void publish_outputs(const rclcpp::Time & now);
  void publish_fault_transitions(const rclcpp::Time & now);

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  std::unique_ptr<flight::AttitudeControlSystem> system_;
  std::unique_ptr<GncParameterBridge> bridge_;
  ThrusterGeometryLoader loader_;
  flight::ControlResult last_result_{};

  flight::VehicleSensors sensors_{};
  flight::AttitudeCommand command_{};
  bool have_attitude_{false};

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Quaternion>::SharedPtr attitude_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3>::SharedPtr rate_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3>::SharedPtr torque_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3>::SharedPtr torque_cmg_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3>::SharedPtr torque_thr_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3>::SharedPtr cmg_momentum_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_rate_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_pub_;
  rclcpp_lifecycle::LifecyclePublisher<GncModeState>::SharedPtr mode_pub_;
  rclcpp_lifecycle::LifecyclePublisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp_lifecycle::LifecyclePublisher<SubsystemHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp_lifecycle::LifecyclePublisher<FaultEvent>::SharedPtr fault_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr attitude_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr rate_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr pose_ref_sub_;
  rclcpp::Subscription<WorldState>::SharedPtr world_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr unload_sub_;

  rclcpp::Service<SetActuationMode>::SharedPtr set_mode_srv_;
  rclcpp::Client<RegisterSubsystem>::SharedPtr register_client_;

  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  std::string subsystem_name_{"gnc"};
  double step_rate_hz_{10.0};
  double heartbeat_rate_hz_{1.0};
  rclcpp::Time last_step_time_;
  bool first_step_{true};
  bool params_dirty_{false};
};
}  // namespace nodes
}  // namespace ssos_gnc

#endif  // SSOS_GNC__NODES__ATTITUDE_CONTROL_NODE_HPP_
