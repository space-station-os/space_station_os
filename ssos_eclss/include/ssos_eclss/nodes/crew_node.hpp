#ifndef SSOS_ECLSS__NODES__CREW_NODE_HPP_
#define SSOS_ECLSS__NODES__CREW_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "space_station_interfaces/msg/heat_load_array.hpp"

#include "ssos_eclss/crew/crew_simulator.hpp"
#include "ssos_eclss/nodes/node_support.hpp"

// Lifecycle node for the crew: metabolic gas and water loads over the crew
// day for the cabin and the water processor, and metabolic heat for the
// thermal network.

namespace ssos_eclss
{
namespace nodes
{

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

  const crew::CrewOutputs & last_outputs() const {return last_outputs_;}

private:
  void step();
  crew::CrewParams build_params() const;
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  crew::CrewSimulator crew_;
  crew::CrewOutputs last_outputs_{};

  using FloatPub = rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr;
  FloatPub co2_pub_;
  FloatPub o2_pub_;
  FloatPub latent_pub_;
  FloatPub urine_pub_;
  FloatPub potable_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    telemetry_pub_;
  rclcpp_lifecycle::LifecyclePublisher<space_station_interfaces::msg::HeatLoadArray>::SharedPtr
    heat_pub_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  InjectedFaults drills_;
  HealthReporter health_;

  double step_rate_hz_{1.0};
  double initial_hour_{10.0};       // crew time at sim time 0 [h]
  double metabolic_scale_{1.0};
  int crew_size_{4};
  std::string thermal_node_{"base_link"};
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__CREW_NODE_HPP_
