#ifndef SSOS_ECLSS__NODES__SABATIER_NODE_HPP_
#define SSOS_ECLSS__NODES__SABATIER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include "ssos_eclss/nodes/eps_power_client.hpp"
#include "ssos_eclss/nodes/node_support.hpp"
#include "ssos_eclss/sabatier/sabatier_system.hpp"

// Lifecycle node wrapping the Sabatier CO2-reduction reactor. Closes the ECLSS
// loop: CO2 desorbed by the ARS + H2 from the OGS -> CH4 (vented) + H2O (to the
// WRS). CO2 in excess of the H2-limited reactable amount is vented to space.

namespace ssos_eclss
{
namespace nodes
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SabatierNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SabatierNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
  void step();
  void publish(const rclcpp::Time & now, double co2_in_mol_s, double supply);

  std::unique_ptr<sabatier::SabatierSystem> sabatier_;
  sabatier::SabatierResult last_result_{};

  // Feed streams: ARS CO2 removal [kg/s] and OGS O2 [kg/s] (2 H2 per O2);
  // nothing arrives from a silent unit.
  LatchedValue co2_in_{kKgDayToKgS, 0.0};
  LatchedValue ogs_o2_{kKgDayToKgS, 0.0};

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr water_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    telemetry_pub_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;

  StepClock clock_;
  InjectedFaults drills_;
  HealthReporter health_;

  double step_rate_hz_{1.0};
  double min_reactor_temp_k_{550.0};
  bool stepped_{false};   // conversion is only judged after a reactor step
  bool enable_auto_faults_{true};

  // Power from ssos_eps; full_power_w_ is the demand at full function.
  EpsPowerClient power_{this, "eclss_sabatier", 2};
  bool eps_coupling_{true};
  double full_power_w_{0.0};
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__SABATIER_NODE_HPP_
