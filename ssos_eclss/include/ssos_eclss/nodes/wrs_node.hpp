#ifndef SSOS_ECLSS__NODES__WRS_NODE_HPP_
#define SSOS_ECLSS__NODES__WRS_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/srv/register_subsystem.hpp"

#include "ssos_eclss/wrs/water_recovery_system.hpp"

// Lifecycle node wrapping the Water Recovery System physics.

namespace ssos_eclss
{
namespace nodes
{

using RegisterSubsystem = space_station_interfaces::srv::RegisterSubsystem;
using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using FaultEvent = space_station_interfaces::msg::FaultEvent;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class WrsNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit WrsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  double last_conductivity_us() const { return last_result_.product_conductivity_us; }

private:
  void step();
  void register_with_manager();
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  std::unique_ptr<wrs::WaterRecoverySystem> wrs_;
  wrs::WrsResult last_result_{};

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr water_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    telemetry_pub_;
  rclcpp_lifecycle::LifecyclePublisher<SubsystemHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp_lifecycle::LifecyclePublisher<FaultEvent>::SharedPtr fault_pub_;
  rclcpp::Client<RegisterSubsystem>::SharedPtr register_client_;
  // Closed-loop: clean product water recovered by the Sabatier reactor, fed in
  // as an extra (distillate-quality) condensate stream.
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sabatier_water_sub_;
  double sabatier_water_kg_s_{0.0};
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  double step_rate_hz_{1.0};
  double urine_kg_day_{6.0};
  double condensate_kg_day_{3.0};
  double potable_limit_us_{100.0};
  rclcpp::Time last_step_time_;
  bool first_step_{true};
  bool enable_auto_faults_{false};  // faults injected explicitly, not auto-tripped
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__WRS_NODE_HPP_
