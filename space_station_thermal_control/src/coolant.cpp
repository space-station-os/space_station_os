#include "space_station_thermal_control/coolant.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace space_station_thermal_control
{

CoolantManager::CoolantManager()
: Node("internal_coolant"),
  tank_capacity_(90.0),
  ammonia_volume_(90.0),
  ammonia_temp_(-20.0),
  ammonia_pressure_(101325.0),
  heater_on_(false),
  heater_logged_(false),
  loop_mass_kg_(25.0),
  initial_temperature_(10.0),
  current_temperature_(15.0),
  control_step_counter_(0),
  water_acquired_(false),
  water_request_pending_(false)
{
  water_client_ = this->create_client<space_station_eclss::srv::RequestProductWater>("/wrs/product_water_request");


  ammonia_server_ = this->create_service<space_station_thermal_control::srv::CoolantFlow>(
    "/tcs/request_ammonia", std::bind(&CoolantManager::handle_ammonia, this, _1, _2));

  thermal_state_server_ = this->create_service<space_station_thermal_control::srv::InternalLoop>(
    "/tcs/loop_a/thermal_state", std::bind(&CoolantManager::handle_thermal_state_request, this, _1, _2));

  fill_loops_server_ = this->create_service<std_srvs::srv::Trigger>(
    "/tcs/fill_loops", std::bind(&CoolantManager::handle_fill_loops, this, _1, _2));

  loop_temp_pub_ = this->create_publisher<space_station_thermal_control::msg::InternalLoopStatus>("/tcs/internal_loop_heat", 10);

  loop_temp_sub_ = this->create_subscription<space_station_thermal_control::msg::ExternalLoopStatus>(
    "/tcs/external_loop_a/status", 10, std::bind(&CoolantManager::apply_heat_reduction, this, _1));

  status_pub_ = this->create_publisher<space_station_thermal_control::msg::TankStatus>("/tcs/ammonia_status", 10);

  heatflow_server_ = this->create_service<space_station_thermal_control::srv::NodeHeatFlow>(
    "/internal_loop_cooling", std::bind(&CoolantManager::handle_heatflow, this, _1, _2));

  control_timer_ = this->create_wall_timer(
    std::chrono::seconds(5), std::bind(&CoolantManager::control_cycle, this));
}

void CoolantManager::handle_fill_loops(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  request_water();
  response->success = true;
  response->message = "Internal loops filled with water.";
}

void CoolantManager::request_water()
{
  if (!water_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(this->get_logger(), "[FILL] Product Water service not available.");
    return;
  }

  auto req = std::make_shared<space_station_eclss::srv::RequestProductWater::Request>();
  req->amount = 100.0;  // request in liters

  water_future_ = water_client_->async_send_request(req);
  water_request_pending_ = true;

  RCLCPP_INFO(this->get_logger(), "[FILL] Requested %.2f L potable water from WRS...", req->amount);
}


void CoolantManager::apply_heat_reduction(
  const space_station_thermal_control::msg::ExternalLoopStatus::SharedPtr msg)
{
  double deltaT = msg->loop_inlet_temp - msg->loop_outlet_temp;
  double before = current_temperature_;
  current_temperature_ = std::max(current_temperature_ - deltaT, 10.0);

  double loop_a_before = before + 1.0;
  double loop_b_before = before + 0.5;
  double loop_a_after = current_temperature_ + 1.0;
  double loop_b_after = current_temperature_ + 0.5;

  RCLCPP_INFO(this->get_logger(),
    "[EXCHANGE] ΔT=%.2f°C | Avg: %.2f→%.2f°C | A: %.2f→%.2f°C | B: %.2f→%.2f°C",
    deltaT, before, current_temperature_,
    loop_a_before, loop_a_after,
    loop_b_before, loop_b_after);
}

void CoolantManager::handle_ammonia(
  const std::shared_ptr<space_station_thermal_control::srv::CoolantFlow::Request> request,
  std::shared_ptr<space_station_thermal_control::srv::CoolantFlow::Response> response)
{
  double req = request->requested_volume;
  if (ammonia_volume_ >= req) {
    ammonia_volume_ -= req;
    response->granted = true;
    response->status_msg = "Ammonia granted.";
  } else {
    response->granted = false;
    response->status_msg = "Insufficient ammonia.";
  }
  response->current_temperature = ammonia_temp_;
}

void CoolantManager::handle_thermal_state_request(
  const std::shared_ptr<space_station_thermal_control::srv::InternalLoop::Request> request,
  std::shared_ptr<space_station_thermal_control::srv::InternalLoop::Response> response)
{
  if (!request->status) {
    response->loop_capacity = 0.0;
    response->temperature = 0.0;
    response->heat_transferred = 0.0;
    return;
  }

  const double Cp = 4.186;
  double delta_T = current_temperature_ - initial_temperature_;
  delta_T = std::max(0.0, delta_T);
  double Q = loop_mass_kg_ * Cp * delta_T;

  response->loop_capacity = loop_mass_kg_;
  response->temperature = current_temperature_;
  response->heat_transferred = Q;
}

void CoolantManager::handle_heatflow(
  const std::shared_ptr<space_station_thermal_control::srv::NodeHeatFlow::Request> request,
  std::shared_ptr<space_station_thermal_control::srv::NodeHeatFlow::Response> response)
{
  double avg_temp = request->heat_flow;  // This is actually average temperature
  const double Cp = 4.186;
  double deltaT = avg_temp - initial_temperature_;
  deltaT = std::max(0.0, deltaT);

  double Q = loop_mass_kg_ * 1000.0 * Cp * deltaT;
  double boosted_heat = Q * 1.2;
  current_temperature_ = std::max(initial_temperature_, current_temperature_ + boosted_heat / (loop_mass_kg_ * 1000.0 * Cp));

  RCLCPP_INFO(this->get_logger(),
    "[COOLING SERVICE] Avg Temp=%.2f°C | ΔT=%.2f°C | Heat=%.2f J | Boosted=%.2f J | T=%.2f°C",
    avg_temp, deltaT, Q, boosted_heat, current_temperature_);

  response->success = true;
  response->message = "Heat applied to internal loop.";
}

void CoolantManager::publish_loop_temperatures()
{
  double loop_a_temp = current_temperature_ + 1.0;
  double loop_b_temp = current_temperature_ + 0.5;
  rclcpp::Time now = this->now();

  space_station_thermal_control::msg::InternalLoopStatus msg;
  msg.loop_a.temperature = loop_a_temp;
  msg.loop_a.variance = 0.0;
  msg.loop_a.header.stamp = now;

  msg.loop_b.temperature = loop_b_temp;
  msg.loop_b.variance = 0.0;
  msg.loop_b.header.stamp = now;

  loop_temp_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(),
    "[TEMP] A: %.2f°C | B: %.2f°C | Avg: %.2f°C",
    loop_a_temp, loop_b_temp, current_temperature_);
}

void CoolantManager::control_cycle()
{
  control_step_counter_++;

  if (!water_acquired_ && control_step_counter_ % 5 == 0 && !water_request_pending_) {
    request_water();
  }

    if (water_request_pending_) {
    auto future_status = water_future_.wait_for(std::chrono::seconds(0));
    if (future_status == std::future_status::ready) {
      auto resp = water_future_.get();
      water_request_pending_ = false;

      if (resp->success && resp->water_granted > 0.0) {
        water_acquired_ = true;
        current_temperature_ += 2.5;

        publish_timer_ = this->create_wall_timer(
          std::chrono::seconds(1), std::bind(&CoolantManager::publish_loop_temperatures, this));

        RCLCPP_INFO(this->get_logger(),
          "[LOOP FILL] %.2fL granted | Temp: %.2f°C",
          resp->water_granted, current_temperature_);
      } else {
        RCLCPP_WARN(this->get_logger(),
          "[FILL] Water request failed: %s",
          resp->message.c_str());
      }
    }
  }


  if (ammonia_temp_ < -10.0 && !heater_on_) {
    heater_on_ = true;
    RCLCPP_INFO(this->get_logger(), "[HEATER] Turning ON (%.2f°C)", ammonia_temp_);
  } else if (ammonia_temp_ > -5.0 && heater_on_) {
    heater_on_ = false;
    RCLCPP_INFO(this->get_logger(), "[HEATER] Turning OFF (%.2f°C)", ammonia_temp_);
  }

  if (heater_on_) {
    ammonia_temp_ += 1.5;
    RCLCPP_DEBUG(this->get_logger(), "[HEATER] Ammonia: %.2f°C", ammonia_temp_);
  }

  ammonia_pressure_ = 101325.0 + (ammonia_temp_ + 20.0) * 1500.0;

  space_station_thermal_control::msg::TankStatus tank_status;
  tank_status.tank_capacity = tank_capacity_;
  tank_status.tank_temperature.temperature = ammonia_temp_;
  tank_status.tank_temperature.variance = 0.0;
  tank_status.tank_temperature.header.stamp = this->now();
  tank_status.tank_pressure.fluid_pressure = ammonia_pressure_;
  tank_status.tank_pressure.variance = 0.0;
  tank_status.tank_pressure.header.stamp = this->now();
  tank_status.tank_heater_on = heater_on_;
  status_pub_->publish(tank_status);
}

}  // namespace space_station_thermal_control

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<space_station_thermal_control::CoolantManager>());
  rclcpp::shutdown();
  return 0;
}
