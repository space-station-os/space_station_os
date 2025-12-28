#ifndef space_station_thermal_control__RADIATOR_CONTROLLER_HPP_
#define space_station_thermal_control__RADIATOR_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "space_station_interfaces/srv/vent_heat.hpp"

namespace space_station_thermal_control
{

class RadiatorController : public rclcpp::Node
{
public:
  RadiatorController();

private:
  void handle_vent_request(
    const std::shared_ptr<space_station_interfaces::srv::VentHeat::Request>,
    std::shared_ptr<space_station_interfaces::srv::VentHeat::Response>);

  rclcpp::Service<space_station_interfaces::srv::VentHeat>::SharedPtr vent_service_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_pub_;
};

}  // namespace space_station_thermal_control

#endif  // space_station_thermal_control__RADIATOR_CONTROLLER_HPP_
