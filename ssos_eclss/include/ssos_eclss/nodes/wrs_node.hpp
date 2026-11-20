#ifndef SSOS_ECLSS__NODES__WRS_NODE_HPP_
#define SSOS_ECLSS__NODES__WRS_NODE_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include "ssos_eclss/nodes/eps_power_client.hpp"
#include "ssos_eclss/nodes/node_support.hpp"
#include "ssos_eclss/wrs/water_recovery_system.hpp"

// Lifecycle node for the Water Recovery System and the station water
// inventory. Urine collects in the wastewater tank and is processed by the
// UPA in batches; UPA distillate, CHX condensate and Sabatier water are
// polished by the WPA into the potable tank, which the crew and the OGS draw.

namespace ssos_eclss
{
namespace nodes
{

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

  double potable_kg() const {return potable_kg_;}
  double wastewater_kg() const {return wastewater_kg_;}
  bool upa_processing() const {return upa_processing_;}

private:
  void step();
  void apply_drill();
  void advance(double dt);
  void collect_urine(double dt);
  void draw_potable(double dt);
  void hold_for_wpa(double kg);
  void publish(const rclcpp::Time & now, double supply);
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  std::unique_ptr<wrs::WaterRecoverySystem> wrs_;
  wrs::WrsResult last_result_{};
  double urine_processed_kg_s_{0.0};
  double off_spec_kg_s_{0.0};
  bool bed_exhausted_{false};

  // Streams [kg/s]. Crew streams keep their last value if the crew node goes
  // quiet; a silent cabin, Sabatier or OGS delivers or draws nothing.
  LatchedValue urine_{kKgDayToKgS, std::nullopt};
  LatchedValue crew_demand_{kKgDayToKgS, std::nullopt};
  LatchedValue condensate_{kKgDayToKgS, 0.0};
  LatchedValue sabatier_water_{kKgDayToKgS, 0.0};
  LatchedValue ogs_demand_{kKgDayToKgS, 0.0};

  // Inventory [kg]. Off-spec WPA product waits in the WPA waste tank and is
  // reprocessed once the product is back in spec.
  double wastewater_kg_{0.0};
  double potable_kg_{0.0};
  double wpa_waste_kg_{0.0};
  double overflow_kg_{0.0};       // water the full tanks could not take
  bool upa_processing_{false};

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr water_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr potable_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr wastewater_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    telemetry_pub_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  StepClock clock_;
  InjectedFaults drills_;
  HealthReporter health_;

  double step_rate_hz_{1.0};
  double potable_limit_us_{100.0};
  double wastewater_capacity_kg_{22.0};
  double potable_capacity_kg_{1000.0};
  double initial_potable_kg_{400.0};
  double initial_wastewater_kg_{8.0};
  double max_urine_process_kg_day_{13.6};
  double upa_start_fraction_{0.70};
  double upa_stop_fraction_{0.05};
  double low_supply_days_{7.0};
  double wpa_reprocess_kg_day_{10.0};
  double wpa_waste_capacity_kg_{100.0};
  bool enable_auto_faults_{true};

  // Power from ssos_eps; full_power_w_ is the demand at full function.
  EpsPowerClient power_{this, "eclss_wrs", 1};
  bool eps_coupling_{true};
  double full_power_w_{0.0};
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__WRS_NODE_HPP_
