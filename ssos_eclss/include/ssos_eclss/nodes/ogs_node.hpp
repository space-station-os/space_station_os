#ifndef SSOS_ECLSS__NODES__OGS_NODE_HPP_
#define SSOS_ECLSS__NODES__OGS_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/msg/world_state.hpp"
#include "space_station_interfaces/srv/register_subsystem.hpp"

#include "ssos_eclss/ogs/oxygen_generator_system.hpp"

// Lifecycle node wrapping the Oxygen Generation System physics.

namespace ssos_eclss
{
namespace nodes
{

using WorldState = space_station_interfaces::msg::WorldState;
using RegisterSubsystem = space_station_interfaces::srv::RegisterSubsystem;
using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using FaultEvent = space_station_interfaces::msg::FaultEvent;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class OgsNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit OgsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  double last_o2_kg_day() const { return last_result_.o2_production_kg_day; }

private:
  void step();
  void register_with_manager();
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  std::unique_ptr<ogs::OxygenGeneratorSystem> ogs_;
  ogs::OgsResult last_result_{};

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr o2_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr water_demand_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    telemetry_pub_;
  rclcpp_lifecycle::LifecyclePublisher<SubsystemHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp_lifecycle::LifecyclePublisher<FaultEvent>::SharedPtr fault_pub_;
  rclcpp::Client<RegisterSubsystem>::SharedPtr register_client_;
  // Feedwater available from the WRS potable bus [kg].
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr potable_available_sub_;
  double potable_available_kg_{0.0};
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  double step_rate_hz_{1.0};
  double stack_current_a_{27.0};
  double o2_required_kg_day_{2.3};
  rclcpp::Time last_step_time_;
  bool first_step_{true};
  bool enable_auto_faults_{false};  // faults injected explicitly, not auto-tripped
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__OGS_NODE_HPP_
