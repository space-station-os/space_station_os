#ifndef SPACE_STATION_CONTROL__TELEOP_NODE_HPP_
#define SPACE_STATION_CONTROL__TELEOP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "space_station_control/msg/thrusters_cmd.hpp"

class TeleopNode : public rclcpp::Node {
public:
  TeleopNode();

private:
  void input_loop();
  rclcpp::Publisher<space_station_control::msg::ThrustersCmd>::SharedPtr command_pub_;
};

#endif  // SPACE_STATION_CONTROL__TELEOP_NODE_HPP_
