#include "space_station_thermal_control/external_loop.hpp"

using std::placeholders::_1;

namespace space_station_thermal_control {

ExternalLoopA::ExternalLoopA()
: Node("external_loop_a"),
  ammonia_temp_(0.0),
  ammonia_filled_(false),
  awaiting_ammonia_response_(false)
{
  internal_sub_ = this->create_subscription<space_station_thermal_control::msg::InternalLoopStatus>(
    "/tcs/internal_loop_heat", 10,
    std::bind(&ExternalLoopA::interface_heat_exchanger, this, _1));

  ammonia_client_ = this->create_client<space_station_thermal_control::srv::CoolantFlow>("/tcs/request_ammonia");

  loop_status_pub_ = this->create_publisher<space_station_thermal_control::msg::ExternalLoopStatus>(
    "/tcs/external_loop_a/status", 10);

  retry_timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    std::bind(&ExternalLoopA::try_ammonia_refill, this));

  radiator_client_ = this->create_client<space_station_thermal_control::srv::VentHeat>("/tcs/radiator_a/vent_heat");

  RCLCPP_INFO(this->get_logger(), "[INIT] External Loop A Node Ready.");

  // First attempt to fill
  try_ammonia_refill();
}

void ExternalLoopA::try_ammonia_refill()
{
  if (ammonia_filled_ || awaiting_ammonia_response_) {
    return;
  }

  if (!ammonia_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(this->get_logger(), "[AMMONIA] Service not available.");
    return;
  }

  auto req = std::make_shared<space_station_thermal_control::srv::CoolantFlow::Request>();
  req->requested_volume = 5.0;

  awaiting_ammonia_response_ = true;

  auto future = ammonia_client_->async_send_request(req,
    [this](rclcpp::Client<space_station_thermal_control::srv::CoolantFlow>::SharedFuture result) {
      awaiting_ammonia_response_ = false;
      auto resp = result.get();

      if (resp->granted) {
        ammonia_temp_ = resp->current_temperature;
        ammonia_filled_ = true;
        RCLCPP_INFO(this->get_logger(), "[AMMONIA] Filled 5.0 kg at %.2f°C", ammonia_temp_);
      } else {
        ammonia_filled_ = false;
        RCLCPP_WARN(this->get_logger(), "[AMMONIA] Request denied: %s", resp->status_msg.c_str());
      }
    });
}

void ExternalLoopA::interface_heat_exchanger(const space_station_thermal_control::msg::InternalLoopStatus::SharedPtr msg)
{
  if (!ammonia_filled_) {
    RCLCPP_WARN(this->get_logger(), "[WAIT] No ammonia available. Skipping heat exchange.");
    return;
  }

  double loop_temp = msg->loop_a.temperature;
  if (loop_temp < 45.0) {
    RCLCPP_INFO(this->get_logger(), "[SKIP] Loop A temperature %.2f°C below threshold (45°C).", loop_temp);
    return;
  }

  // Simulate heat exchange
  const double received_heat = 1200.0;
  const double Cp_ammonia = 4.7;
  const double mass = 5.0;

  double deltaT = received_heat / (mass * Cp_ammonia);
  double ammonia_temp_after = ammonia_temp_ + deltaT;
  double loop_temp_after = loop_temp - deltaT;

  space_station_thermal_control::msg::ExternalLoopStatus status;
  status.received_heat_kj = received_heat;
  status.loop_inlet_temp = loop_temp;
  status.loop_outlet_temp = loop_temp_after;
  status.ammonia_inlet_temp = ammonia_temp_;
  status.ammonia_outlet_temp = ammonia_temp_after;

  loop_status_pub_->publish(status);

  RCLCPP_INFO(this->get_logger(),
    "[EX LOOP] Q=%.2f kJ | Loop: %.2f→%.2f°C | Ammonia: %.2f→%.2f°C",
    received_heat, loop_temp, loop_temp_after, ammonia_temp_, ammonia_temp_after);

  // Mark ammonia as used, refill will be attempted by retry_timer
  ammonia_temp_ = ammonia_temp_after;
  ammonia_filled_ = false;
  if (radiator_client_->wait_for_service(std::chrono::milliseconds(500))) {
    auto req = std::make_shared<space_station_thermal_control::srv::VentHeat::Request>();
    req->excess_heat = received_heat;

    radiator_client_->async_send_request(req,
        [this](rclcpp::Client<space_station_thermal_control::srv::VentHeat>::SharedFuture future) {
          auto resp = future.get();
          if (resp->success) {
            RCLCPP_INFO(this->get_logger(), "[RADIATOR] Heat vented successfully.");
          } else {
            RCLCPP_WARN(this->get_logger(), "[RADIATOR] Failed to vent heat: %s", resp->message.c_str());
          }
        });
  } else {
    RCLCPP_WARN(this->get_logger(), "[RADIATOR] Service unavailable for heat venting.");
  }

}

}  // namespace space_station_thermal_control

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<space_station_thermal_control::ExternalLoopA>());
  rclcpp::shutdown();
  return 0;
}
