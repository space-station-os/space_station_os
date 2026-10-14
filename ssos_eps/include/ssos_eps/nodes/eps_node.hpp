#ifndef SSOS_EPS__NODES__EPS_NODE_HPP_
#define SSOS_EPS__NODES__EPS_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "space_station_interfaces/msg/bcdu_status.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/msg/world_state.hpp"
#include "space_station_interfaces/srv/register_subsystem.hpp"

#include "ssos_eps/nodes/eps_parameter_bridge.hpp"
#include "ssos_eps/system/eps_system.hpp"

// Lifecycle node wrapping the EPS generation and storage physics (solar
// arrays, SSU, BCDUs and battery ORUs).

namespace ssos_eps
{
namespace nodes
{

using BCDUStatus = space_station_interfaces::msg::BCDUStatus;
using FaultEvent = space_station_interfaces::msg::FaultEvent;
using RegisterSubsystem = space_station_interfaces::srv::RegisterSubsystem;
using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using WorldState = space_station_interfaces::msg::WorldState;
using Float64 = std_msgs::msg::Float64;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class EpsNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EpsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  const system::EpsState & last_state() const {return last_state_;}

private:
  template<typename MsgT>
  using Pub = typename rclcpp_lifecycle::LifecyclePublisher<MsgT>::SharedPtr;

  void step();
  void publish_telemetry(const rclcpp::Time & now);
  void publish_faults(const rclcpp::Time & now);
  void apply_fault_hooks();
  void register_with_manager();
  void on_world_state(const WorldState::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  std::unique_ptr<EpsParameterBridge> bridge_;
  std::unique_ptr<system::EpsSystem> eps_;
  system::EpsState last_state_{};

  Pub<diagnostic_msgs::msg::DiagnosticArray> telemetry_pub_;
  Pub<SubsystemHeartbeat> heartbeat_pub_;
  Pub<FaultEvent> fault_pub_;
  Pub<BCDUStatus> bcdu_pub_;
  std::vector<Pub<BCDUStatus>> channel_pubs_;
  std::vector<Pub<sensor_msgs::msg::BatteryState>> oru_pubs_;
  Pub<Float64> soc_pub_;
  Pub<Float64> power_balance_pub_;
  Pub<Float64> array_power_pub_;
  Pub<Float64> ssu_voltage_pub_;
  Pub<Float64> ssu_power_pub_;
  Pub<Float64> ssu_current_pub_;
  Pub<Float64> panel_temp_pub_;
  Pub<Float64> beta_pub_;

  rclcpp::Subscription<WorldState>::SharedPtr world_state_sub_;
  rclcpp::Client<RegisterSubsystem>::SharedPtr register_client_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  double step_rate_hz_{1.0};
  double load_demand_w_{30000.0};
  bool enable_auto_faults_{true};
  bool params_dirty_{false};
  int failed_oru_{-1};
  int faulted_bcdu_{-1};

  double solar_flux_w_m2_{1361.0};
  bool in_eclipse_{false};

  bool low_soc_reported_{false};
  bool load_unserved_reported_{false};
  bool bcdu_fault_reported_{false};
  bool oru_failed_reported_{false};

  rclcpp::Time last_step_time_;
  bool first_step_{true};
};

}  // namespace nodes
}  // namespace ssos_eps

#endif  // SSOS_EPS__NODES__EPS_NODE_HPP_
