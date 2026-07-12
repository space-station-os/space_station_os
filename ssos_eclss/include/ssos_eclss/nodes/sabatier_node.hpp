#ifndef SSOS_ECLSS__NODES__SABATIER_NODE_HPP_
#define SSOS_ECLSS__NODES__SABATIER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/srv/register_subsystem.hpp"

#include "ssos_eclss/sabatier/sabatier_system.hpp"

// Lifecycle node wrapping the Sabatier CO2-reduction reactor. Closes the ECLSS
// loop: CO2 desorbed by the ARS + H2 from the OGS -> CH4 (vented) + H2O (to the
// WRS). CO2 in excess of the H2-limited reactable amount is vented to space.

namespace ssos_eclss
{
namespace nodes
{

using RegisterSubsystem = space_station_interfaces::srv::RegisterSubsystem;
using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using FaultEvent = space_station_interfaces::msg::FaultEvent;
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
  void register_with_manager();

  std::unique_ptr<sabatier::SabatierSystem> sabatier_;
  sabatier::SabatierResult last_result_{};

  // Closed-loop inputs (latched from topics).
  double co2_available_kg_s_{0.0};  // from ARS desorption
  double h2_available_mol_s_{0.0};  // derived from OGS O2 (2 H2 per O2)

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr water_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    telemetry_pub_;
  rclcpp_lifecycle::LifecyclePublisher<SubsystemHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp_lifecycle::LifecyclePublisher<FaultEvent>::SharedPtr fault_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ars_co2_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ogs_o2_sub_;
  rclcpp::Client<RegisterSubsystem>::SharedPtr register_client_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;

  double step_rate_hz_{1.0};
  rclcpp::Time last_step_time_;
  bool first_step_{true};
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__SABATIER_NODE_HPP_
