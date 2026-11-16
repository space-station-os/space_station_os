#ifndef SSOS_GNC__NODES__GNC_PLANT_NODE_HPP_
#define SSOS_GNC__NODES__GNC_PLANT_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/msg/world_state.hpp"

#include "ssos_gnc/nodes/gnc_parameter_bridge.hpp"
#include "ssos_gnc/plant/disturbance_torques.hpp"
#include "ssos_gnc/plant/rigid_body.hpp"
#include "ssos_gnc/plant/sensor_error_model.hpp"

namespace ssos_gnc
{

namespace nodes
{

using WorldState = space_station_interfaces::msg::WorldState;
using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class GncPlantNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit GncPlantNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  const plant::RigidBodyState & plant_state() const {return body_->state();}

private:
  void step();
  void on_gimbal_rate(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void on_cmg_torque(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void on_thruster_torque(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void on_world_state(const WorldState::SharedPtr msg);

  std::unique_ptr<plant::RigidBody> body_;
  std::unique_ptr<plant::DisturbanceTorques> disturbances_;
  std::unique_ptr<plant::SensorErrorModel> sensors_;
  std::unique_ptr<GncParameterBridge> bridge_;

  common::Vector4 gimbal_rate_cmd_{common::Vector4::Zero()};
  common::Vector3 cmg_torque_cmd_{common::Vector3::Zero()};
  common::Vector3 thruster_torque_cmd_{common::Vector3::Zero()};
  plant::EnvironmentConditions env_{};

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Quaternion>::SharedPtr attitude_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3>::SharedPtr rate_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3>::SharedPtr disturbance_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3>::SharedPtr momentum_pub_;
  rclcpp_lifecycle::LifecyclePublisher<SubsystemHeartbeat>::SharedPtr heartbeat_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_rate_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cmg_torque_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr thruster_torque_sub_;
  rclcpp::Subscription<WorldState>::SharedPtr world_state_sub_;

  rclcpp::TimerBase::SharedPtr step_timer_;

  std::string subsystem_name_{"gnc_plant"};
  double step_rate_hz_{50.0};
  rclcpp::Time last_step_time_;
  bool first_step_{true};
};
}  // namespace nodes
}  // namespace ssos_gnc

#endif  // SSOS_GNC__NODES__GNC_PLANT_NODE_HPP_
