#ifndef SSOS_ECLSS__NODES__CABIN_NODE_HPP_
#define SSOS_ECLSS__NODES__CABIN_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/srv/register_subsystem.hpp"

#include "ssos_eclss/cabin/cabin_atmosphere.hpp"
#include "ssos_eclss/cabin/crew_metabolic_model.hpp"
#include "ssos_eclss/cabin/leak_model.hpp"

// Lifecycle node simulating the cabin atmosphere driven by crew metabolic loads
// and leakage. Publishes ppCO2, O2 fraction and total pressure telemetry.

namespace ssos_eclss
{
namespace nodes
{

using RegisterSubsystem = space_station_interfaces::srv::RegisterSubsystem;
using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using FaultEvent = space_station_interfaces::msg::FaultEvent;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CabinNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CabinNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  double last_co2_ppm() const { return last_co2_ppm_; }

private:
  void step();
  void register_with_manager();
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  std::unique_ptr<cabin::CabinAtmosphere> atmosphere_;
  std::unique_ptr<cabin::CrewMetabolicModel> crew_;
  std::unique_ptr<cabin::LeakModel> leak_;
  double last_co2_ppm_{0.0};

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr co2_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    telemetry_pub_;
  rclcpp_lifecycle::LifecyclePublisher<SubsystemHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp_lifecycle::LifecyclePublisher<FaultEvent>::SharedPtr fault_pub_;
  rclcpp::Client<RegisterSubsystem>::SharedPtr register_client_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Closed-loop coupling: ARS removes CO2 from the cabin, OGS adds O2.
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ars_removal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ogs_o2_sub_;
  double ars_co2_removal_kg_s_{0.0};
  double ogs_o2_kg_s_{0.0};

  double step_rate_hz_{1.0};
  int crew_size_{4};
  double cabin_volume_m3_{100.0};
  double cabin_temp_c_{22.0};
  double co2_alarm_ppm_{7000.0};
  rclcpp::Time last_step_time_;
  bool first_step_{true};
  bool enable_auto_faults_{false};  // faults injected explicitly, not auto-tripped
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__CABIN_NODE_HPP_
