#ifndef SSOS_ECLSS__NODES__CABIN_NODE_HPP_
#define SSOS_ECLSS__NODES__CABIN_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "space_station_interfaces/msg/cabin_atmosphere.hpp"
#include "space_station_interfaces/msg/thermal_node_data_array.hpp"

#include "ssos_eclss/cabin/atmosphere_control.hpp"
#include "ssos_eclss/cabin/cabin_atmosphere.hpp"
#include "ssos_eclss/cabin/leak_model.hpp"
#include "ssos_eclss/nodes/node_support.hpp"

// Lifecycle node owning the cabin atmosphere. Crew loads, ARS CO2 removal,
// OGS O2, leakage, the condensing heat exchanger and pressure control drive a
// well-mixed cabin; the result goes out on /ssos/cabin/atmosphere for the
// other ECLSS units and the GUI.

namespace ssos_eclss
{
namespace nodes
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using CabinAtmosphereMsg = space_station_interfaces::msg::CabinAtmosphere;

class CabinNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CabinNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  const cabin::CabinAtmosphere & atmosphere() const {return *atmosphere_;}

private:
  void step();
  void apply_drill();
  void publish(const rclcpp::Time & now);
  void check_health(const rclcpp::Time & now);
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  std::unique_ptr<cabin::CabinAtmosphere> atmosphere_;
  std::unique_ptr<cabin::LeakModel> leak_;
  cabin::AtmosphereControl control_;
  cabin::ControlResult last_control_{};
  double leak_kg_s_{0.0};

  // Mass-balance inputs [kg/s]. Crew loads keep their last value if the crew
  // node goes quiet; a silent ARS or OGS is removing or adding nothing.
  LatchedValue crew_co2_{kKgDayToKgS, std::nullopt};
  LatchedValue crew_o2_{kKgDayToKgS, std::nullopt};
  LatchedValue crew_latent_{kKgDayToKgS, std::nullopt};
  LatchedValue ars_removal_{kKgDayToKgS, 0.0};
  LatchedValue ogs_o2_{kKgDayToKgS, 0.0};
  rclcpp::Subscription<space_station_interfaces::msg::ThermalNodeDataArray>::SharedPtr
    thermal_sub_;

  rclcpp_lifecycle::LifecyclePublisher<CabinAtmosphereMsg>::SharedPtr atmosphere_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr co2_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr condensate_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    telemetry_pub_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  StepClock clock_;
  InjectedFaults drills_;
  HealthReporter health_;

  double step_rate_hz_{1.0};
  double cabin_volume_m3_{388.0};
  double cabin_temp_c_{22.0};
  std::string thermal_node_{"base_link"};   // cabin air temperature from thermal
  double initial_co2_ppm_{2600.0};
  double initial_rh_{0.45};
  double leak_area_m2_{2.0e-8};
  double co2_alarm_ppm_{5300.0};
  double o2_low_pct_{19.5};
  double o2_high_pct_{23.5};
  double pressure_low_kpa_{97.0};
  double pressure_high_kpa_{104.0};
  double rh_low_{0.25};
  double rh_high_{0.75};
  double leak_alarm_kg_day_{3.0};
  double o2_sensor_tolerance_pct_{0.5};
  bool enable_auto_faults_{true};
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__CABIN_NODE_HPP_
