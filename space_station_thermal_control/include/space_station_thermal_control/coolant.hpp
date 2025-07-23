#ifndef space_station_thermal_control__COOLANT_MANAGER_HPP_
#define space_station_thermal_control__COOLANT_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"


#include "space_station_thermal_control/srv/coolant_flow.hpp"
#include "space_station_thermal_control/srv/internal_loop.hpp"
#include "space_station_thermal_control/srv/node_heat_flow.hpp"
#include "space_station_eclss/srv/request_product_water.hpp"
#include "space_station_thermal_control/msg/tank_status.hpp"
#include "space_station_thermal_control/msg/internal_loop_status.hpp"
#include "space_station_thermal_control/msg/external_loop_status.hpp"

namespace space_station_thermal_control
{

class CoolantManager : public rclcpp::Node
{
public:
  CoolantManager();

private:
 
  void request_water();
  void handle_fill_loops(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response>);


  void handle_ammonia(
    const std::shared_ptr<space_station_thermal_control::srv::CoolantFlow::Request>,
    std::shared_ptr<space_station_thermal_control::srv::CoolantFlow::Response>);

  
  void handle_thermal_state_request(
    const std::shared_ptr<space_station_thermal_control::srv::InternalLoop::Request>,
    std::shared_ptr<space_station_thermal_control::srv::InternalLoop::Response>);

  void handle_heatflow(
    const std::shared_ptr<space_station_thermal_control::srv::NodeHeatFlow::Request>,
    std::shared_ptr<space_station_thermal_control::srv::NodeHeatFlow::Response>);

  void publish_loop_temperatures();
  void apply_heat_reduction(
    const space_station_thermal_control::msg::ExternalLoopStatus::SharedPtr);


  void control_cycle();

  rclcpp::Client<space_station_eclss::srv::RequestProductWater>::SharedPtr water_client_;
  rclcpp::Service<space_station_thermal_control::srv::CoolantFlow>::SharedPtr ammonia_server_;
  rclcpp::Service<space_station_thermal_control::srv::InternalLoop>::SharedPtr thermal_state_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fill_loops_server_;
  rclcpp::Service<space_station_thermal_control::srv::NodeHeatFlow>::SharedPtr heatflow_server_;

  rclcpp::Publisher<space_station_thermal_control::msg::InternalLoopStatus>::SharedPtr loop_temp_pub_;
  rclcpp::Subscription<space_station_thermal_control::msg::ExternalLoopStatus>::SharedPtr loop_temp_sub_;
  rclcpp::Publisher<space_station_thermal_control::msg::TankStatus>::SharedPtr status_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Client<space_station_eclss::srv::RequestProductWater>::SharedFuture water_future_;
  bool water_request_pending_ = false;

  // === System state ===
  double tank_capacity_;
  double ammonia_volume_;
  double ammonia_temp_;
  double ammonia_pressure_;
  bool heater_on_;
  bool heater_logged_;  

  double loop_mass_kg_;
  double initial_temperature_;
  double current_temperature_;

 
  int control_step_counter_;
  bool water_acquired_;
};

}  // namespace space_station_thermal_control

#endif  // space_station_thermal_control__COOLANT_MANAGER_HPP_