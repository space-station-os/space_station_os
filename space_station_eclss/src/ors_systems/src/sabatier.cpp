#include "space_station_eclss/ors_system/sabatier.h"
#include <cmath>
#include <algorithm>

Sabatier::Sabatier()
    : Node("sabatier_reactor"),
      co2_mass_(0.0),
      h2_mass_(0.0),
      methane_yield_(0.0),
      water_yield_(0.0),
      reaction_efficiency_(0.0),
      reactor_temperature_(300.0),
      reactor_health_status_("OK") {

  co2_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/co2_storage", 10,
      std::bind(&Sabatier::co2_callback, this, std::placeholders::_1));

  h2_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/h2_storage", 10,
      std::bind(&Sabatier::h2_callback, this, std::placeholders::_1));

  grey_water_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/grey_water", 10);

  methane_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/vented_methane", 10);

  health_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/sabatier_health_status", 10);

  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&Sabatier::run_reactor, this));

  RCLCPP_INFO(this->get_logger(), "Sabatier Reactor Node initialized.");
}

void Sabatier::co2_callback(const std_msgs::msg::Float64::SharedPtr msg) {
  co2_mass_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "CO2 Input: %.2f g", co2_mass_);
}

void Sabatier::h2_callback(const std_msgs::msg::Float64::SharedPtr msg) {
  h2_mass_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "H2 Input: %.2f g", h2_mass_);
}

void Sabatier::run_reactor() {
  if (co2_mass_ <= 0.0 || h2_mass_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Waiting for both CO2 and H2 inputs...");
    return;
  }

  double co2_moles = co2_mass_ / 44.01;
  double h2_moles = h2_mass_ / 2.016;
  double limiting_moles = std::min(co2_moles, h2_moles / 4.0);

  methane_yield_ = limiting_moles * 16.04;
  water_yield_ = limiting_moles * 2.0 * 18.015;
  reaction_efficiency_ = (co2_moles > 0.0) ? (limiting_moles / co2_moles) * 100.0 : 0.0;

  // Simulate reactor temperature variation
  reactor_temperature_ += ((rand() % 100) - 50) * 0.01;  // fluctuate within ±0.5°C
  if (reactor_temperature_ > 500.0 || reactor_temperature_ < 200.0) {
    reactor_health_status_ = "FAULT: Temperature out of range";
  } else {
    reactor_health_status_ = "OK";
  }

  std_msgs::msg::Float64 water_msg;
  water_msg.data = water_yield_;
  grey_water_publisher_->publish(water_msg);

  std_msgs::msg::Float64 methane_msg;
  methane_msg.data = methane_yield_;
  methane_publisher_->publish(methane_msg);

  std_msgs::msg::String health_msg;
  health_msg.data = reactor_health_status_;
  health_publisher_->publish(health_msg);

  RCLCPP_INFO(this->get_logger(),
              "[Sabatier] CH4: %.2f g, H2O: %.2f g, Temp: %.2f C, Eff: %.2f%%, Health: %s",
              methane_yield_, water_yield_, reactor_temperature_,
              reaction_efficiency_, reactor_health_status_.c_str());

  co2_mass_ = 0.0;
  h2_mass_ = 0.0;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Sabatier>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
