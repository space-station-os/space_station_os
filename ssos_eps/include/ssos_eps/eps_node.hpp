#ifndef SSOS_EPS__EPS_NODE_HPP_
#define SSOS_EPS__EPS_NODE_HPP_

#include <memory>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "space_station_interfaces/msg/world_state.hpp"
#include "space_station_interfaces/srv/register_subsystem.hpp"

#include "ssos_eps/eps_model.hpp"

namespace ssos_eps
{

using FaultEvent = space_station_interfaces::msg::FaultEvent;
using RegisterSubsystem = space_station_interfaces::srv::RegisterSubsystem;
using SubsystemHeartbeat = space_station_interfaces::msg::SubsystemHeartbeat;
using WorldState = space_station_interfaces::msg::WorldState;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class EpsNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EpsNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;

  const EpsState & last_state() const
  {
    return last_state_;
  }

private:
  void step();
  void register_with_manager();
  void on_world_state(const WorldState::SharedPtr msg);

  std::unique_ptr<EpsModel> model_;
  EpsState last_state_;

  rclcpp_lifecycle::LifecyclePublisher<
    diagnostic_msgs::msg::DiagnosticArray>::SharedPtr telemetry_pub_;

  rclcpp_lifecycle::LifecyclePublisher<
    SubsystemHeartbeat>::SharedPtr heartbeat_pub_;

  rclcpp_lifecycle::LifecyclePublisher<
    FaultEvent>::SharedPtr fault_pub_;

  rclcpp::Subscription<WorldState>::SharedPtr world_state_sub_;
  rclcpp::Client<RegisterSubsystem>::SharedPtr register_client_;
  rclcpp::TimerBase::SharedPtr step_timer_;

  double step_rate_hz_{1.0};

  double solar_flux_w_m2_{1361.0};
  bool in_eclipse_{false};

  double solar_array_area_m2_{100.0};
  double solar_array_efficiency_{0.30};
  double load_demand_w_{20000.0};
  double battery_capacity_wh_{50000.0};
  double initial_battery_soc_{0.80};
  double low_soc_threshold_{0.20};

  bool enable_auto_faults_{true};
  bool low_soc_fault_published_{false};

  rclcpp::Time last_step_time_;
  bool first_step_{true};
};

}  // namespace ssos_eps

#endif  // SSOS_EPS__EPS_NODE_HPP_
