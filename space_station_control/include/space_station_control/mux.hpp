#ifndef SPACE_STATION_CONTROL__MUX_NODE_HPP_
#define SPACE_STATION_CONTROL__MUX_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "space_station_control/msg/thrusters_cmd.hpp"
#include "std_msgs/msg/float64.hpp"
#include <map>
#include <string>
#include <vector>




class MuxNode : public rclcpp::Node {
public:
  MuxNode();

private:
  void command_callback(const space_station_control::msg::ThrustersCmd::SharedPtr msg);

  rclcpp::Subscription<space_station_control::msg::ThrustersCmd>::SharedPtr command_sub_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> thruster_pubs_;
};

#endif  // SPACE_STATION_CONTROL__MUX_NODE_HPP_
