#include "space_station_eps/mbsu_device.hpp"
#include <algorithm>
#include <limits>

namespace space_station_eps
{
constexpr double CHANNEL_TIMEOUT_SEC = 3.0;
using namespace std::chrono_literals;
MbsuNode::MbsuNode(const rclcpp::NodeOptions &options)
: Node("mbsu_node", options)
{
  num_channels_ = this->declare_parameter("num_channels", 12);

  for (int i = 0; i < num_channels_; ++i) {
    battery_subs_[i] = this->create_subscription<std_msgs::msg::Float64>(
      "/mbsu/channel_" + std::to_string(i) + "/voltage",
      rclcpp::SensorDataQoS(),
      [this, i](const std_msgs::msg::Float64::SharedPtr msg) {
        batteryCallback(i, msg);
      });
  }
  

  ddcu_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ddcu/input_voltage", 10);
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/eps/diagnostics", 10);
  timer_ = this->create_wall_timer(
      2s,  
      std::bind(&MbsuNode::selectHealthyChannels, this)
    );
  RCLCPP_INFO(this->get_logger(), "MBSU Node initialized with %d channels", num_channels_);
}

void MbsuNode::batteryCallback(int channel_id, const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  channel_voltage_[channel_id] = msg->data;
  last_update_[channel_id] = this->now();
}

std::pair<int, int> MbsuNode::selectHealthyChannels()
{
  std::lock_guard<std::mutex> lock(mtx_);
  std::vector<std::pair<int, float>> healthy;

  rclcpp::Time now = this->now();

  for (const auto &[id, voltage] : channel_voltage_) {
    auto it = last_update_.find(id);
    if (it == last_update_.end()) continue;

    double age = (now - it->second).seconds();
    if (age < CHANNEL_TIMEOUT_SEC && voltage > 120.0f) {
      healthy.emplace_back(id, voltage);
    } else if (age >= CHANNEL_TIMEOUT_SEC) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Channel %d is stale (last update %.1f s ago)", id, age);
    }
  }

  
  std::sort(healthy.begin(), healthy.end(), [](const auto &a, const auto &b) {
    return a.second > b.second;
  });

  if (healthy.size() < 2) {
    RCLCPP_ERROR(this->get_logger(), "Insufficient healthy channels (found %ld)", healthy.size());

    diagnostic_msgs::msg::DiagnosticStatus diag;
    diag.name = "MBSU";
    diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag.message = "Less than two healthy channels available";

    for (int i = 0; i < num_channels_; ++i) {
      std::string key = "channel_" + std::to_string(i);
      double age = std::numeric_limits<double>::infinity();
      auto it = last_update_.find(i);
      if (it != last_update_.end()) {
        age = (now - it->second).seconds();
      }

      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = age < CHANNEL_TIMEOUT_SEC ? "OK" : "STALE";
      diag.values.push_back(kv);

    }


    diag_pub_->publish(diag);
    return {-1, -1};
  }

  float combined_voltage = (healthy[0].second + healthy[1].second) / 2.0;
  std_msgs::msg::Float64 msg;
  msg.data = combined_voltage;
  ddcu_pub_->publish(msg);

  

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    "Publishing combined voltage %.2f V from channels %d and %d",
    combined_voltage, healthy[0].first, healthy[1].first);

  return {healthy[0].first, healthy[1].first};
}

}  // namespace space_station_eps


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<space_station_eps::MbsuNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}