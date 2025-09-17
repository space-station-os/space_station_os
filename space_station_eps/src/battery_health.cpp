#include "space_station_eps/battery_health.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

using std::placeholders::_1;
using std::placeholders::_2;

BatteryManager::BatteryManager() : Node("battery_manager_node")
{
  this->declare_parameter<int>("num_channels", 12);
  this->declare_parameter<std::string>("battery_config", "config/battery_config.yaml");
  this->get_parameter("num_channels", num_channels_);

  std::string config_file;
  this->get_parameter("battery_config", config_file);
  std::string config_path = ament_index_cpp::get_package_share_directory("space_station_eps") + "/" + config_file;
  YAML::Node config = YAML::LoadFile(config_path);

  int total_orus = num_channels_ * 2;
  RCLCPP_INFO(this->get_logger(), "Initializing BatteryManager with %d ORUs", total_orus);

  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/eps/diagnostics", 10);

  for (int i = 0; i < total_orus; ++i)
  {
    auto unit = std::make_shared<BatteryUnit>();
    unit->id = "battery_bms_" + std::to_string(i);
    unit->voltage = BATTERY_MAX_VOLTAGE;
    unit->discharging = false;
    unit->location = config[unit->id] ? config[unit->id].as<std::string>() : "unknown";

    unit->publisher = this->create_publisher<sensor_msgs::msg::BatteryState>(
      "/battery/" + unit->id + "/health", 10);

    unit->discharge_service = this->create_service<std_srvs::srv::Trigger>(
      "/battery/" + unit->id + "/discharge",
      [this, unit](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        handle_discharge_request(unit, request, response);
      });

    unit->charge_service = this->create_service<std_srvs::srv::Trigger>(
      "/battery/" + unit->id + "/charge",
      [this, unit](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        handle_charge_request(unit, request, response);
      });

    batteries_.push_back(unit);
  }

  // Simulate charging/discharging every second
  sim_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      for (auto &unit : batteries_) {
        simulate_unit(unit);
        publish_battery_state(unit);
      }
    });
}

void BatteryManager::simulate_unit(std::shared_ptr<BatteryUnit> unit)
{
  if (unit->discharging) {
    unit->voltage -= 1.5;
    if (unit->voltage <= BATTERY_CRITICAL_VOLTAGE) {
      unit->voltage = BATTERY_CRITICAL_VOLTAGE;
      unit->discharging = false;
      diagnostic_msgs::msg::DiagnosticStatus diag;
      diag.name = unit->id;
      diag.level = diag.ERROR;
      diag.message = "Voltage dropped below 70V. Stopping discharge.";
      diag.hardware_id = unit->location;
      diag_pub_->publish(diag);
    }
  } else {
    unit->voltage += 1.5;
    if (unit->voltage > BATTERY_MAX_VOLTAGE)
      unit->voltage = BATTERY_MAX_VOLTAGE;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
    "%s voltage: %.2f V (%s)",
    unit->id.c_str(),
    unit->voltage,
    unit->discharging ? "discharging" : "charging");
}

void BatteryManager::handle_discharge_request(
  std::shared_ptr<BatteryUnit> unit,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (unit->voltage <= BATTERY_CRITICAL_VOLTAGE) {
    response->success = false;
    response->message = "Voltage too low for discharge.";
    return;
  }
  unit->discharging = true;
  response->success = true;
  response->message = "Battery " + unit->id + " is now discharging.";
}

void BatteryManager::handle_charge_request(
  std::shared_ptr<BatteryUnit> unit,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (unit->voltage >= BATTERY_MAX_VOLTAGE) {
    response->success = false;
    response->message = "Battery already sufficiently charged.";
    return;
  }
  unit->discharging = false;
  response->success = true;
  response->message = "Battery " + unit->id + " is now charging.";
}

void BatteryManager::publish_battery_state(const std::shared_ptr<BatteryUnit>& unit)
{
  sensor_msgs::msg::BatteryState b;
  b.header.stamp = this->now();
  b.header.frame_id = unit->id;
  b.voltage = unit->voltage;
  b.current = unit->discharging ? -5.0f : 4.0f;
  b.charge = unit->voltage / BATTERY_MAX_VOLTAGE * 100.0f;
  b.capacity = 100.0f;
  b.percentage = unit->voltage / BATTERY_MAX_VOLTAGE;
  b.power_supply_status = unit->discharging ? b.POWER_SUPPLY_STATUS_DISCHARGING : b.POWER_SUPPLY_STATUS_CHARGING;
  b.power_supply_health = (unit->voltage < BATTERY_CRITICAL_VOLTAGE)
    ? b.POWER_SUPPLY_HEALTH_DEAD
    : (unit->voltage < BATTERY_WARNING_VOLTAGE ? b.POWER_SUPPLY_HEALTH_OVERHEAT : b.POWER_SUPPLY_HEALTH_GOOD);
  b.power_supply_technology = b.POWER_SUPPLY_TECHNOLOGY_LION;
  b.present = true;
  b.location = unit->location;
  b.serial_number = unit->id;

  b.cell_voltage.resize(38);
  b.cell_temperature.resize(38);
  float avg_cell_voltage = unit->voltage / 38.0f;
  for (int i = 0; i < 38; ++i) {
    b.cell_voltage[i] = avg_cell_voltage + static_cast<float>(rand() % 100 - 50) / 1000.0f;
    b.cell_temperature[i] = 25.0f + static_cast<float>(rand() % 100 - 50) / 10.0f;
  }

  unit->publisher->publish(b);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BatteryManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
