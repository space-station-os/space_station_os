#ifndef SSOS_ECLSS__NODES__CREW_NODE_HPP_
#define SSOS_ECLSS__NODES__CREW_NODE_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/srv/register_subsystem.hpp"

#include "ssos_eclss/crew/crew_simulator.hpp"

// Astronaut (crew) simulator lifecycle node. Publishes the crew's metabolic gas
// loads (to the cabin) and water streams (to the WRS + potable bus), driven by
// a diurnal activity schedule over sim time.

namespace ssos_eclss
{
namespace nodes
{

using RegisterSubsystem = space_station_interfaces::srv::RegisterSubsystem;
using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CrewNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CrewNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  const crew::CrewOutputs & last_outputs() const { return last_outputs_; }

private:
  void step();
  void register_with_manager();
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);
  crew::CrewParams build_params() const;

  std::unique_ptr<crew::CrewSimulator> crew_;
  crew::CrewOutputs last_outputs_{};

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr co2_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr o2_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr latent_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr urine_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr potable_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    telemetry_pub_;
  rclcpp_lifecycle::LifecyclePublisher<SubsystemHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::Client<RegisterSubsystem>::SharedPtr register_client_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  double step_rate_hz_{1.0};
  double day_length_s_{86400.0};   // length of one sim "day" [s]
  double metabolic_scale_{1.0};    // global multiplier on gas/latent rates
  int crew_size_{4};
  double drink_kg_day_{2.20};
  double flush_kg_day_{0.30};
  double urine_kg_day_{1.20};
  double feces_water_kg_day_{0.15};
  double trash_water_kg_day_{0.20};
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__CREW_NODE_HPP_
