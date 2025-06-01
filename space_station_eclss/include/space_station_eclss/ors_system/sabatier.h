#ifndef SPACE_STATION_ECLSS__SABATIER_H_
#define SPACE_STATION_ECLSS__SABATIER_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

class Sabatier : public rclcpp::Node
{
public:
  Sabatier();

private:
  // Callbacks
  void co2_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void h2_callback(const std_msgs::msg::Float64::SharedPtr msg);

  // Main reactor processing
  void run_reactor();

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr co2_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr h2_subscriber_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr grey_water_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr methane_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Internal state
  double co2_mass_;
  double h2_mass_;
  double methane_yield_;
  double water_yield_;
  double reaction_efficiency_;
  double reactor_temperature_;
  std::string reactor_health_status_;
};

#endif  // SPACE_STATION_ECLSS__SABATIER_H_
