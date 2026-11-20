#ifndef SSOS_ECLSS__NODES__OGS_NODE_HPP_
#define SSOS_ECLSS__NODES__OGS_NODE_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "space_station_interfaces/msg/cabin_atmosphere.hpp"

#include "ssos_eclss/nodes/eps_power_client.hpp"
#include "ssos_eclss/nodes/node_support.hpp"
#include "ssos_eclss/ogs/o2_controller.hpp"
#include "ssos_eclss/ogs/oxygen_generator_system.hpp"

// Lifecycle node for the Oxygen Generation System. The stack current follows
// the cabin ppO2 controller; feedwater comes from the WRS potable tank and
// power from ssos_eps.

namespace ssos_eclss
{
namespace nodes
{

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

  double last_o2_kg_day() const {return last_result_.o2_production_kg_day;}
  double commanded_current_a() const {return controller_.current();}

private:
  void step();
  void publish(const rclcpp::Time & now, double supply);
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  std::unique_ptr<ogs::OxygenGeneratorSystem> ogs_;
  ogs::O2PressureController controller_;
  ogs::OgsResult last_result_{};

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr o2_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr water_demand_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    telemetry_pub_;
  rclcpp::Subscription<space_station_interfaces::msg::CabinAtmosphere>::SharedPtr cabin_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr potable_sub_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  StepClock clock_;
  InjectedFaults drills_;
  HealthReporter health_;

  double step_rate_hz_{1.0};
  double initial_current_a_{17.5};
  double cabin_o2_kpa_{-1.0};        // < 0 until the cabin reports
  rclcpp::Clock wall_{RCL_STEADY_TIME};
  rclcpp::Time cabin_stamp_{0, 0, RCL_STEADY_TIME};
  double commanded_a_{0.0};          // controller current after the EPS supply
  double potable_kg_{-1.0};          // < 0 until the WRS reports (feedwater unlimited)
  bool enable_auto_faults_{true};

  // Power from ssos_eps; full_power_w_ is the demand at full function.
  EpsPowerClient power_{this, "eclss_ogs", 1};
  bool eps_coupling_{true};
  double full_power_w_{0.0};
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__OGS_NODE_HPP_
