#include "space_station_thermal_control/coolant.hpp"

#include <algorithm>
#include <utility>

using std::placeholders::_1;
using std::placeholders::_2;

namespace space_station_thermal_control
{

CoolantManager::CoolantManager(const rclcpp::NodeOptions & options)
: Node("internal_coolant", options),
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
  water_request_pending_(false),
  rng_(std::random_device{}())
{
  // ---- Parameters ----
  this->declare_parameter<double>("cp_j_per_kg_c",           4186.0);   // water Cp
  this->declare_parameter<double>("loop_mass_kg",            loop_mass_kg_);
  this->declare_parameter<double>("initial_temperature_c",   initial_temperature_);
  this->declare_parameter<double>("min_internal_temp_c",     10.0);
  this->declare_parameter<double>("max_internal_temp_c",     95.0);
  this->declare_parameter<double>("control_period_s",        5.0);
  this->declare_parameter<double>("publish_period_s",        1.0);
  this->declare_parameter<double>("water_request_liters",    100.0);
  this->declare_parameter<int>("refresh_period_cycles",      7);
  this->declare_parameter<double>("refresh_volume_l",        50.0);
  this->declare_parameter<int>("recycle_every_goals",        10);
  this->declare_parameter<double>("recycle_volume_l",        50.0);
  this->declare_parameter<std::string>("heatflow_input_mode","temp_c"); // or "joules"
  this->declare_parameter<double>("heat_boost_gain",         1.2);

  // Ammonia / pressure / heater
  this->declare_parameter<double>("pressure_base_pa",        101325.0);
  this->declare_parameter<double>("pressure_gain_pa_per_c",  1500.0);
  this->declare_parameter<double>("pressure_ref_offset_c",   20.0);
  this->declare_parameter<double>("max_ammonia_pressure_pa", 3.0e6);
  this->declare_parameter<double>("heater_on_below_c",       -10.0);
  this->declare_parameter<double>("heater_off_above_c",      -5.0);
  this->declare_parameter<double>("heater_warm_rate_c_per_cycle", 1.5);

  // Fault injection
  this->declare_parameter<bool>("enable_failures",           false);
  this->declare_parameter<double>("p_drop_water_req",        0.0);
  this->declare_parameter<double>("p_drop_ammonia_grant",    0.0);

  // Fetch params
  cp_j_per_kg_c_           = this->get_parameter("cp_j_per_kg_c").as_double();
  loop_mass_kg_            = this->get_parameter("loop_mass_kg").as_double();
  initial_temperature_     = this->get_parameter("initial_temperature_c").as_double();
  min_internal_temp_c_     = this->get_parameter("min_internal_temp_c").as_double();
  max_internal_temp_c_     = this->get_parameter("max_internal_temp_c").as_double();
  control_period_s_        = this->get_parameter("control_period_s").as_double();
  publish_period_s_        = this->get_parameter("publish_period_s").as_double();
  water_request_liters_    = this->get_parameter("water_request_liters").as_double();
  refresh_period_cycles_   = this->get_parameter("refresh_period_cycles").as_int();
  refresh_volume_l_        = this->get_parameter("refresh_volume_l").as_double();
  recycle_every_goals_     = this->get_parameter("recycle_every_goals").as_int();
  recycle_volume_l_        = this->get_parameter("recycle_volume_l").as_double();
  heatflow_input_mode_     = this->get_parameter("heatflow_input_mode").as_string();
  heat_boost_gain_         = this->get_parameter("heat_boost_gain").as_double();

  pressure_base_pa_        = this->get_parameter("pressure_base_pa").as_double();
  pressure_gain_pa_per_c_  = this->get_parameter("pressure_gain_pa_per_c").as_double();
  pressure_ref_offset_c_   = this->get_parameter("pressure_ref_offset_c").as_double();
  max_ammonia_pressure_pa_ = this->get_parameter("max_ammonia_pressure_pa").as_double();
  heater_on_below_c_       = this->get_parameter("heater_on_below_c").as_double();
  heater_off_above_c_      = this->get_parameter("heater_off_above_c").as_double();
  heater_warm_rate_c_per_cycle_ = this->get_parameter("heater_warm_rate_c_per_cycle").as_double();

  enable_failures_         = this->get_parameter("enable_failures").as_bool();
  p_drop_water_req_        = this->get_parameter("p_drop_water_req").as_double();
  p_drop_ammonia_grant_    = this->get_parameter("p_drop_ammonia_grant").as_double();

  // ---- Interfaces ----
  water_client_ = this->create_client<space_station_eclss::srv::RequestProductWater>(
    "/wrs/product_water_request");

  ammonia_server_ = this->create_service<space_station_thermal_control::srv::CoolantFlow>(
    "/tcs/request_ammonia", std::bind(&CoolantManager::handle_ammonia, this, _1, _2));

  thermal_state_server_ = this->create_service<space_station_thermal_control::srv::InternalLoop>(
    "/tcs/loop_a/thermal_state", std::bind(&CoolantManager::handle_thermal_state_request, this, _1, _2));

  fill_loops_server_ = this->create_service<std_srvs::srv::Trigger>(
    "/tcs/fill_loops", std::bind(&CoolantManager::handle_fill_loops, this, _1, _2));

  loop_temp_pub_ = this->create_publisher<space_station_thermal_control::msg::InternalLoopStatus>(
    "/tcs/internal_loop_heat", 10);

  loop_temp_sub_ = this->create_subscription<space_station_thermal_control::msg::ExternalLoopStatus>(
    "/tcs/external_loop_a/status", 10, std::bind(&CoolantManager::apply_heat_reduction, this, _1));

  status_pub_ = this->create_publisher<space_station_thermal_control::msg::TankStatus>(
    "/tcs/ammonia_status", 10);

  heat_available_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/tcs/internal_loop_heat_j", 10);

  grey_water_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/grey_water", 10);

  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "/thermals/diagnostics", 10);

  heatflow_server_ = this->create_service<space_station_thermal_control::srv::NodeHeatFlow>(
    "/internal_loop_cooling", std::bind(&CoolantManager::handle_heatflow, this, _1, _2));

  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(control_period_s_)),
    std::bind(&CoolantManager::control_cycle, this));
}

// -----------------------------------------------------------

void CoolantManager::publish_diag(int8_t level, const std::string & message)
{
  diagnostic_msgs::msg::DiagnosticStatus d;
  d.level = level;
  d.name = "CoolantManager";
  d.message = message;
  diag_pub_->publish(d);
}

// -----------------------------------------------------------

void CoolantManager::handle_fill_loops(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  request_water();
  response->success = true;
  response->message = "Requested potable water to fill internal loops.";
}

// Overload that uses parameter default
void CoolantManager::request_water()
{
  request_water(water_request_liters_);
}

void CoolantManager::request_water(double amount_l)
{
  if (water_request_pending_) {
    return;
  }

  if (!water_client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_WARN(this->get_logger(), "[FILL] Product Water service not available.");
    publish_diag(diagnostic_msgs::msg::DiagnosticStatus::WARN, "WRS service unavailable");
    return;
  }

  if (enable_failures_ && uni01_(rng_) < p_drop_water_req_) {
    RCLCPP_WARN(this->get_logger(), "[FILL][Injected] Water request dropped.");
    publish_diag(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Injected failure: dropped water request");
    return;
  }

  auto req = std::make_shared<space_station_eclss::srv::RequestProductWater::Request>();
  req->amount = amount_l;

  water_future_ = water_client_->async_send_request(req);
  water_request_pending_ = true;

  RCLCPP_INFO(this->get_logger(), "[FILL] Requested %.2f L potable water from WRS...", req->amount);
  publish_diag(diagnostic_msgs::msg::DiagnosticStatus::OK, "Requested water: " + std::to_string(amount_l) + " L");
}

// -----------------------------------------------------------

void CoolantManager::apply_heat_reduction(
  const space_station_thermal_control::msg::ExternalLoopStatus::SharedPtr msg)
{
  const double deltaT = msg->loop_inlet_temp - msg->loop_outlet_temp;
  const double before = current_temperature_;
  current_temperature_ = clamp_internal_temp(current_temperature_ - deltaT);

  const double loop_a_before = before + 1.0;
  const double loop_b_before = before + 0.5;
  const double loop_a_after  = current_temperature_ + 1.0;
  const double loop_b_after  = current_temperature_ + 0.5;

  RCLCPP_INFO(this->get_logger(),
    "[EXCHANGE] ΔT=%.2f°C | Avg: %.2f->%.2f°C | A: %.2f->%.2f°C | B: %.2f->%.2f°C",
    deltaT, before, current_temperature_,
    loop_a_before, loop_a_after,
    loop_b_before, loop_b_after);
}

// -----------------------------------------------------------

void CoolantManager::handle_ammonia(
  const std::shared_ptr<space_station_thermal_control::srv::CoolantFlow::Request> request,
  std::shared_ptr<space_station_thermal_control::srv::CoolantFlow::Response> response)
{
  const double req = request->requested_volume;

  if (enable_failures_ && uni01_(rng_) < p_drop_ammonia_grant_) {
    response->granted = false;
    response->status_msg = "Injected failure: ammonia grant dropped";
    response->current_temperature = ammonia_temp_;
    publish_diag(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Injected failure: ammonia grant dropped");
    return;
  }

  if (ammonia_volume_ >= req) {
    ammonia_volume_ -= req;
    response->granted = true;
    response->status_msg = "Ammonia granted.";
    publish_diag(diagnostic_msgs::msg::DiagnosticStatus::OK, "Ammonia granted: " + std::to_string(req) + " L");
  } else {
    response->granted = false;
    response->status_msg = "Insufficient ammonia.";
    publish_diag(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Ammonia depleted (requested " + std::to_string(req) + " L)");
  }
  response->current_temperature = ammonia_temp_;
}

// -----------------------------------------------------------

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

  const double delta_T = std::max(0.0, current_temperature_ - initial_temperature_);
  const double Q_j = loop_mass_kg_ * cp_j_per_kg_c_ * delta_T; // Joules

  response->loop_capacity = loop_mass_kg_;      // kg
  response->temperature = current_temperature_; // °C
  response->heat_transferred = Q_j;             // Joules (standardized)
}

// -----------------------------------------------------------

void CoolantManager::handle_heatflow(
  const std::shared_ptr<space_station_thermal_control::srv::NodeHeatFlow::Request> request,
  std::shared_ptr<space_station_thermal_control::srv::NodeHeatFlow::Response> response)
{
  // Interpret input according to mode
  double Q_j = 0.0;

  if (heatflow_input_mode_ == "joules") {
    // Treat request->heat_flow as Joules
    Q_j = std::max(0.0, request->heat_flow);
  } else { // "temp_c" (default/back-compat)
    const double avg_temp_c = request->heat_flow;
    const double deltaT = std::max(0.0, avg_temp_c - current_temperature_); // from current state
    Q_j = loop_mass_kg_ * cp_j_per_kg_c_ * deltaT;
  }

  // Optional gain/fudge factor
  const double Q_eff_j = std::max(0.0, heat_boost_gain_ * Q_j);

  // Apply heat to internal loop temperature
  const double before = current_temperature_;
  current_temperature_ = clamp_internal_temp(
    current_temperature_ + (Q_eff_j / (loop_mass_kg_ * cp_j_per_kg_c_)));

  // Publish immediate routing of heat to external loop (as available internal heat)
  std_msgs::msg::Float64 jmsg;
  jmsg.data = Q_eff_j;
  heat_available_pub_->publish(jmsg);

  // Count a "goal" (successful heat acceptance)
  ++successful_goals_;

  RCLCPP_INFO(this->get_logger(),
    "[HEATFLOW] Q=%.2f J (eff=%.2f) | T: %.2f->%.2f°C | goals=%d",
    Q_j, Q_eff_j, before, current_temperature_, successful_goals_);

  publish_diag(diagnostic_msgs::msg::DiagnosticStatus::OK,
               "Applied heat J=" + std::to_string(Q_eff_j));

  // Recycle after N "goals"
  if (recycle_every_goals_ > 0 && (successful_goals_ % recycle_every_goals_) == 0) {
    std_msgs::msg::Float64 grey;
    grey.data = recycle_volume_l_;
    grey_water_pub_->publish(grey);
    publish_diag(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 "Recycled internal loop water after goals=" + std::to_string(successful_goals_));
    request_water(recycle_volume_l_);
  }

  response->success = true;
  response->message = "Heat applied to internal loop.";
}

// -----------------------------------------------------------

void CoolantManager::publish_loop_temperatures()
{
  const double loop_a_temp = current_temperature_ + 1.0;
  const double loop_b_temp = current_temperature_ + 0.5;
  const rclcpp::Time now = this->now();

  space_station_thermal_control::msg::InternalLoopStatus msg;
  msg.loop_a.temperature = loop_a_temp;
  msg.loop_a.variance = 0.0;
  msg.loop_a.header.stamp = now;

  msg.loop_b.temperature = loop_b_temp;
  msg.loop_b.variance = 0.0;
  msg.loop_b.header.stamp = now;

  loop_temp_pub_->publish(msg);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "[TEMP] A: %.2f°C | B: %.2f°C | Avg: %.2f°C",
    loop_a_temp, loop_b_temp, current_temperature_);
}

// -----------------------------------------------------------

double CoolantManager::clamp_internal_temp(double t) const
{
  if (t < min_internal_temp_c_) return min_internal_temp_c_;
  if (t > max_internal_temp_c_) return max_internal_temp_c_;
  return t;
}

void CoolantManager::update_ammonia_pressure_and_safety()
{
  // Pressure model and safety checks
  ammonia_pressure_ = pressure_base_pa_
    + (ammonia_temp_ + pressure_ref_offset_c_) * pressure_gain_pa_per_c_;

  if (ammonia_pressure_ > max_ammonia_pressure_pa_) {
    if (heater_on_) {
      heater_on_ = false;
      publish_diag(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Over-pressure detected. Forcing heater OFF.");
      RCLCPP_ERROR(this->get_logger(),
        "[SAFETY] Over-pressure %.0f Pa > %.0f Pa. Heater OFF.",
        ammonia_pressure_, max_ammonia_pressure_pa_);
    }
  }
}

// -----------------------------------------------------------

void CoolantManager::control_cycle()
{
  control_step_counter_++;

  // Auto-fill attempt until first acquisition
  if (!water_acquired_ && (control_step_counter_ % 5 == 0) && !water_request_pending_) {
    request_water();
  }

  // Periodic refresh of internal loop water
  if (refresh_period_cycles_ > 0 &&
      (control_step_counter_ % refresh_period_cycles_) == 0 &&
      water_acquired_ && !water_request_pending_)
  {
    std_msgs::msg::Float64 grey;
    grey.data = refresh_volume_l_;
    grey_water_pub_->publish(grey);
    publish_diag(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 "Periodic refresh: pushed grey water " + std::to_string(refresh_volume_l_) + " L");
    request_water(refresh_volume_l_);
  }

  // Check async water response
  if (water_request_pending_) {
    auto future_status = water_future_.wait_for(std::chrono::seconds(0));
    if (future_status == std::future_status::ready) {
      auto resp = water_future_.get();
      water_request_pending_ = false;

      if (resp->success && resp->water_granted > 0.0) {
        water_acquired_ = true;
        // Slight thermal bump when filling
        current_temperature_ = clamp_internal_temp(current_temperature_ + 2.5);

        if (!publish_timer_started_) {
          publish_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(publish_period_s_)),
            std::bind(&CoolantManager::publish_loop_temperatures, this));
          publish_timer_started_ = true;
        }

        RCLCPP_INFO(this->get_logger(),
          "[LOOP FILL] %.2f L granted | Temp: %.2f°C",
          resp->water_granted, current_temperature_);

        publish_diag(diagnostic_msgs::msg::DiagnosticStatus::OK,
          "Water granted: " + std::to_string(resp->water_granted) + " L");
      } else {
        RCLCPP_WARN(this->get_logger(),
          "[FILL] Water request failed: %s", resp->message.c_str());
        publish_diag(diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "Water request failed: " + resp->message);
      }
    }
  }

  // Heater control (with hysteresis)
  if (ammonia_temp_ < heater_on_below_c_ && !heater_on_) {
    heater_on_ = true;
    RCLCPP_INFO(this->get_logger(), "[HEATER] Turning ON (%.2f°C)", ammonia_temp_);
    publish_diag(diagnostic_msgs::msg::DiagnosticStatus::OK, "Heater ON");
  } else if (ammonia_temp_ > heater_off_above_c_ && heater_on_) {
    heater_on_ = false;
    RCLCPP_INFO(this->get_logger(), "[HEATER] Turning OFF (%.2f°C)", ammonia_temp_);
    publish_diag(diagnostic_msgs::msg::DiagnosticStatus::OK, "Heater OFF");
  }

  if (heater_on_) {
    ammonia_temp_ += heater_warm_rate_c_per_cycle_;
    RCLCPP_DEBUG(this->get_logger(), "[HEATER] Ammonia: %.2f°C", ammonia_temp_);
  }

  // Pressure + safety
  update_ammonia_pressure_and_safety();

  // Publish tank status
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

// Standalone entry point (no components)
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<space_station_thermal_control::CoolantManager>());
  rclcpp::shutdown();
  return 0;
}
