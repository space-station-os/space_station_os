#include "space_station_thermal_control/external_loop.hpp"

#include <functional>
#include <string>

using std::placeholders::_1;

namespace space_station_thermal_control
{

// -------------------- Helpers --------------------
void ExternalLoopA::publish_diag(int8_t level, const std::string & message)
{
  diagnostic_msgs::msg::DiagnosticStatus d;
  d.level = level;                 // OK=0, WARN=1, ERROR=2, STALE=3
  d.name = "ExternalLoopA";        // component name
  d.message = message;
  diag_pub_->publish(d);
}

// -------------------- Ctor --------------------
ExternalLoopA::ExternalLoopA()
: Node("external_loop_a")
{
  // Parameters (with sane defaults)
  this->declare_parameter<double>("request_volume_kg",        request_volume_kg_);        // 5.0
  this->declare_parameter<double>("loop_a_threshold_c",       loop_a_threshold_c_);       // 45.0
  this->declare_parameter<double>("assumed_received_heat_kj", assumed_received_heat_kj_); // 1200.0
  this->declare_parameter<double>("cp_ammonia_kj_per_kg_c",   cp_ammonia_kj_per_kg_c_);   // 4.7

  request_volume_kg_        = this->get_parameter("request_volume_kg").as_double();
  loop_a_threshold_c_       = this->get_parameter("loop_a_threshold_c").as_double();
  assumed_received_heat_kj_ = this->get_parameter("assumed_received_heat_kj").as_double();
  cp_ammonia_kj_per_kg_c_   = this->get_parameter("cp_ammonia_kj_per_kg_c").as_double();

  // Interfaces
  internal_sub_ = this->create_subscription<space_station_thermal_control::msg::InternalLoopStatus>(
    "/tcs/internal_loop_heat", 10,
    std::bind(&ExternalLoopA::interface_heat_exchanger, this, _1));

  ammonia_client_ = this->create_client<space_station_thermal_control::srv::CoolantFlow>(
    "/tcs/request_ammonia");

  radiator_client_ = this->create_client<space_station_thermal_control::srv::VentHeat>(
    "/tcs/radiator_a/vent_heat");

  loop_status_pub_ = this->create_publisher<space_station_thermal_control::msg::ExternalLoopStatus>(
    "/tcs/external_loop_a/status", 10);

  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "/thermals/diagnostics", 10);

  retry_timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    std::bind(&ExternalLoopA::try_ammonia_refill, this));

  RCLCPP_INFO(this->get_logger(), "[INIT] External Loop A Node Ready.");
  publish_diag(diagnostic_msgs::msg::DiagnosticStatus::OK, "ExternalLoopA initialized");

  // Initial attempt to fill ammonia
  try_ammonia_refill();
}

// -------------------- Ammonia Refill --------------------
void ExternalLoopA::try_ammonia_refill()
{
  if (ammonia_filled_ || awaiting_ammonia_response_) {
    return;
  }

  if (!ammonia_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(this->get_logger(), "[AMMONIA] Service not available.");
    publish_diag(diagnostic_msgs::msg::DiagnosticStatus::WARN, "CoolantFlow service unavailable");
    return;
  }

  auto req = std::make_shared<space_station_thermal_control::srv::CoolantFlow::Request>();
  req->requested_volume = request_volume_kg_;  // using "kg" as mass proxy

  awaiting_ammonia_response_ = true;

  auto fut = ammonia_client_->async_send_request(
    req,
    [this](rclcpp::Client<space_station_thermal_control::srv::CoolantFlow>::SharedFuture result)
    {
      awaiting_ammonia_response_ = false;
      auto resp = result.get();

      if (resp->granted) {
        ammonia_temp_ = resp->current_temperature;
        ammonia_filled_ = true;
        RCLCPP_INFO(this->get_logger(), "[AMMONIA] Filled %.2f kg at %.2f°C",
                    request_volume_kg_, ammonia_temp_);
        publish_diag(diagnostic_msgs::msg::DiagnosticStatus::OK,
                     "Ammonia granted: " + std::to_string(request_volume_kg_) +
                     " kg at " + std::to_string(ammonia_temp_) + " °C");
      } else {
        ammonia_filled_ = false;
        RCLCPP_WARN(this->get_logger(), "[AMMONIA] Request denied: %s", resp->status_msg.c_str());
        publish_diag(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                     "Ammonia request denied: " + resp->status_msg);
      }
    }
  );
  (void)fut;
}

// -------------------- Heat Exchanger --------------------
void ExternalLoopA::interface_heat_exchanger(
  const space_station_thermal_control::msg::InternalLoopStatus::SharedPtr msg)
{
  if (!ammonia_filled_) {
    RCLCPP_WARN(this->get_logger(), "[WAIT] No ammonia available. Skipping heat exchange.");
    publish_diag(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                 "Heat exchange skipped: no ammonia available");
    return;
  }

  const double loop_temp = msg->loop_a.temperature;

  if (loop_temp < loop_a_threshold_c_) {
    RCLCPP_INFO(this->get_logger(),
      "[SKIP] Loop A temperature %.2f°C below threshold (%.2f°C).",
      loop_temp, loop_a_threshold_c_);
    publish_diag(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 "Heat exchange not required (LoopA below threshold)");
    return;
  }

  // Modeled heat exchange (kJ)
  const double received_heat_kj = assumed_received_heat_kj_;        // kJ
  const double mass_kg         = request_volume_kg_;                // kg
  const double cp_kj_per_kg_c  = cp_ammonia_kj_per_kg_c_;           // kJ/(kg·°C)

  // ΔT = Q / (m * Cp)
  const double deltaT = received_heat_kj / (mass_kg * cp_kj_per_kg_c);
  const double ammonia_temp_after = ammonia_temp_ + deltaT;
  const double loop_temp_after    = loop_temp - deltaT;

  // Publish external loop status (kept in kJ to match existing message naming)
  space_station_thermal_control::msg::ExternalLoopStatus status;
  status.received_heat_kj   = received_heat_kj;
  status.loop_inlet_temp    = loop_temp;
  status.loop_outlet_temp   = loop_temp_after;
  status.ammonia_inlet_temp = ammonia_temp_;
  status.ammonia_outlet_temp= ammonia_temp_after;
  loop_status_pub_->publish(status);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "[EX LOOP] Q=%.2f kJ | Loop: %.2f->%.2f°C | Ammonia: %.2f->%.2f°C",
    received_heat_kj, loop_temp, loop_temp_after, ammonia_temp_, ammonia_temp_after);

  publish_diag(diagnostic_msgs::msg::DiagnosticStatus::OK,
               "Heat exchange: Q=" + std::to_string(received_heat_kj) +
               " kJ, ΔT=" + std::to_string(deltaT) + " °C");

  // Mark ammonia as used; refill will be attempted by retry_timer
  ammonia_temp_  = ammonia_temp_after;
  ammonia_filled_ = false;

  // Attempt to vent heat via radiator
  if (radiator_client_->wait_for_service(std::chrono::milliseconds(500))) {
    auto req = std::make_shared<space_station_thermal_control::srv::VentHeat::Request>();
    // NOTE: Your current VentHeat srv field name is 'excess_heat' (kJ).
    // If you standardize to Joules later, update both sides consistently.
    req->excess_heat = received_heat_kj;

    auto fut = radiator_client_->async_send_request(
      req,
      [this](rclcpp::Client<space_station_thermal_control::srv::VentHeat>::SharedFuture future)
      {
        try {
          auto resp = future.get();
          if (resp->success) {
            RCLCPP_INFO(this->get_logger(), "[RADIATOR] Heat vented successfully.");
            publish_diag(diagnostic_msgs::msg::DiagnosticStatus::OK, "Radiator vent success");
          } else {
            RCLCPP_WARN(this->get_logger(), "[RADIATOR] Failed to vent heat: %s", resp->message.c_str());
            publish_diag(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                         "Radiator vent failed: " + resp->message);
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "[RADIATOR] VentHeat call threw: %s", e.what());
          publish_diag(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                       std::string("Radiator call exception: ") + e.what());
        }
      }
    );
    (void)fut;
  } else {
    RCLCPP_WARN(this->get_logger(), "[RADIATOR] Service unavailable for heat venting.");
    publish_diag(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Radiator service unavailable");
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
