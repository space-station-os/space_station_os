#include "space_station_control/mux.hpp"

MuxNode::MuxNode() : Node("mux_node") {
  command_sub_ = this->create_subscription<space_station_control::msg::ThrustersCmd>(
    "/thruster_command", 10,
    std::bind(&MuxNode::command_callback, this, std::placeholders::_1)
  );

  // New 16 thruster joint IDs
  std::vector<std::string> thruster_ids = {
    "xn_th_1", "xn_th_2", "xn_th_3", "xn_th_4",
    "xp_th_1", "xp_th_2", "xp_th_3", "xp_th_4",
    "yn_th_1", "yn_th_2",
    "yp_th_1", "yp_th_2",
    "zn_th_1", "zn_th_2",
    "zp_th_1", "zp_th_2"
  };

  for (const auto &id : thruster_ids) {
    auto topic = "/spacestation/thrust/" + id;
    thruster_pubs_[id] = this->create_publisher<std_msgs::msg::Float64>(topic, 10);
  }

  RCLCPP_INFO(this->get_logger(), "Mux node initialized with updated thrusters.");
}

void MuxNode::command_callback(const space_station_control::msg::ThrustersCmd::SharedPtr msg) {
  double thrust = msg->thrust;
  std::string dir = msg->direction;

  // Stop all thrusters before activating a new set
  for (const auto &pair : thruster_pubs_) {
    std_msgs::msg::Float64 zero_msg;
    zero_msg.data = 0.0;
    pair.second->publish(zero_msg);
  }

  std::vector<std::string> active_thrusters;

  // Define motion-specific thruster groups (based on new IDs)
  if (dir == "forward") {
    active_thrusters = { "xp_th_1", "xp_th_2", "xp_th_3", "xp_th_4" };
  } else if (dir == "backward") {
    active_thrusters = { "xn_th_1", "xn_th_2", "xn_th_3", "xn_th_4" };
  } else if (dir == "left") {
    active_thrusters = { "yp_th_1", "yp_th_2" };
  } else if (dir == "right") {
    active_thrusters = { "yn_th_1", "yn_th_2" };
  } else if (dir == "up") {
    active_thrusters = { "zp_th_1", "zp_th_2" };
  } else if (dir == "down") {
    active_thrusters = { "zn_th_1", "zn_th_2" };
  } else if (dir == "yaw_left") {
    active_thrusters = { "xp_th_1", "xp_th_2", "xn_th_3", "xn_th_4" }; 
  } else if (dir == "yaw_right") {
    active_thrusters = { "xn_th_1", "xn_th_2", "xp_th_3", "xp_th_4" };  
  } else if (dir == "halt") {
    RCLCPP_INFO(this->get_logger(), "Halting all thrusters.");
    return;
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown direction command: %s", dir.c_str());
    return;
  }

  for (const auto &thruster : active_thrusters) {
    std_msgs::msg::Float64 msg_out;
    msg_out.data = thrust;
    thruster_pubs_[thruster]->publish(msg_out);
  }

  RCLCPP_INFO(this->get_logger(), "Direction: %s @ %f", dir.c_str(), thrust);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MuxNode>());
  rclcpp::shutdown();
  return 0;
}
