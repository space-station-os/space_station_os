#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <unordered_map>
#include <mutex>

namespace space_station_eps
{

class MbsuNode : public rclcpp::Node
{
public:
  explicit MbsuNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  std::pair<int, int> selectHealthyChannels();

private:
  void batteryCallback(int channel_id, const std_msgs::msg::Float64::SharedPtr msg); 

  int num_channels_;
  std::mutex mtx_;

  std::unordered_map<int, rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> battery_subs_;  
  std::unordered_map<int, float> channel_voltage_;
  std::unordered_map<int, rclcpp::Time> last_update_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ddcu_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
};

}  // namespace space_station_eps
