#include "space_station_thermal_control/radiators.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace space_station_thermal_control
{

RadiatorController::RadiatorController()
: Node("radiator_controller")
{
  vent_service_ = this->create_service<space_station_thermal_control::srv::VentHeat>(
    "/tcs/radiator_a/vent_heat",
    std::bind(&RadiatorController::handle_vent_request, this, _1, _2));

  joint_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/solar_controller/commands", 10);

  RCLCPP_INFO(this->get_logger(), "[INIT] Radiator Controller Node is ready.");
}

void RadiatorController::handle_vent_request(
  const std::shared_ptr<space_station_thermal_control::srv::VentHeat::Request> request,
  std::shared_ptr<space_station_thermal_control::srv::VentHeat::Response> response)
{
  double heat = request->excess_heat;
  RCLCPP_INFO(this->get_logger(), "[VENT] Received %.2f kJ to vent", heat);

  if (heat > 35.0) {
    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data.resize(6);  

    // Alternate target angle slightly
    double base_angle = 1.57;
    static bool toggle = false;
    double delta = toggle ? 0.5 : -1.0;
    toggle = !toggle;

    for (auto &val : cmd_msg.data)
      val = base_angle + delta;

    joint_command_pub_->publish(cmd_msg);

    response->success = true;
    response->message = "Solar panels rotated for thermal radiation.";
  } else {
    response->success = false;
    response->message = "Heat too low to vent.";
  }
}


}  // namespace space_station_thermal_control

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<space_station_thermal_control::RadiatorController>());
  rclcpp::shutdown();
  return 0;
}
