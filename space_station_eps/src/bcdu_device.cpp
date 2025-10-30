#include "space_station_eps/bcdu_device.hpp"

#include <chrono>
#include <thread>
#include <utility>

using namespace std::chrono_literals;

namespace space_station_eps
{

BcduNode::BcduNode(const rclcpp::NodeOptions &options)
: Node("bcdu_node", options)
{
  max_discharge_current_A_ = this->declare_parameter("max_discharge_current_A", 127.0);
  max_charge_current_A_    = this->declare_parameter("max_charge_current_A", 65.0);
  regulation_min_V_        = this->declare_parameter("regulation_min_V", 130.0);
  regulation_max_V_        = this->declare_parameter("regulation_max_V", 180.0);
  regulation_voltage_      = this->declare_parameter("regulation_voltage", 150.0);

  ssu_charge_enter_v_      = this->declare_parameter("ssu_charge_enter_v", 160.0);
  ssu_discharge_enter_v_   = this->declare_parameter("ssu_discharge_enter_v", 152.0);
  ssu_nominal_v_           = this->declare_parameter("ssu_nominal_v", 160.0);

  num_channels_            = this->declare_parameter("num_channels", 12);
  orus_per_channel_        = this->declare_parameter("orus_per_channel", 2);

  const int total_orus = num_channels_ * orus_per_channel_;
  for (int i = 0; i < total_orus; ++i) {
    const std::string base = "/battery/battery_bms_" + std::to_string(i);
    const std::string health_topic = base + "/health";
    const std::string charge_srv   = base + "/charge";
    const std::string discharge_srv= base + "/discharge";

    battery_subs_[i] = this->create_subscription<sensor_msgs::msg::BatteryState>(
      health_topic, rclcpp::SensorDataQoS(),
      [this, i](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        this->batteryCallback(i, msg);
      });

    charge_clients_[i]    = this->create_client<std_srvs::srv::Trigger>(charge_srv);
    discharge_clients_[i] = this->create_client<std_srvs::srv::Trigger>(discharge_srv);
  }

  ssu_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/solar_controller/ssu_voltage_v", 10,
    std::bind(&BcduNode::ssuVoltageCb, this, std::placeholders::_1));

  status_pub_      = this->create_publisher<space_station_interfaces::msg::BCDUStatus>("/bcdu/status", 10);
  diag_pub_        = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/eps/diagnostics", 10);
  for (int ch = 0; ch < num_channels_; ++ch) {
      std::string topic = "/mbsu/channel_" + std::to_string(ch) + "/voltage";
      channel_voltage_pubs_[ch] = this->create_publisher<std_msgs::msg::Float64>(topic, 10);
    }


  RCLCPP_INFO(this->get_logger(), "BCDU Device READY..");
}

void BcduNode::batteryCallback(int id, const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  battery_state_[id] = BatteryInfo{
    msg->voltage,
    (msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING),
    this->now()
  };
}

void BcduNode::ssuVoltageCb(const std_msgs::msg::Float64::SharedPtr msg)
{
  double ssu_v = msg->data;
  ssu_last_update_ = this->now();
  std::string desired;

  if (ssu_v >= ssu_charge_enter_v_) {
    desired = "charge";
  } else if (ssu_v <= ssu_discharge_enter_v_) {
    desired = "discharge";
  } else {
    desired = mode_;  // hold
  }

  if (desired != mode_) {
    if (desired == "charge") {
      enterCharge();
    } else if (desired == "discharge") {
      enterDischarge();
    }
  }

  publishToMbsu(desired == "charge" ? ssu_nominal_v_ : regulation_voltage_);
  publishStatus(desired);
  mode_ = desired;
}

void BcduNode::enterCharge()
{
  publishDiag(diagnostic_msgs::msg::DiagnosticStatus::OK, "BCDU", "Entering CHARGE");
  RCLCPP_INFO(this->get_logger(), "[BCDU] Charging batteries...");

  for (const auto& [id, client] : charge_clients_) {
    if (battery_state_[id].voltage < 120.0f && client->service_is_ready()) {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      client->async_send_request(req);
    }
  }
}

void BcduNode::enterDischarge()
{
  publishDiag(diagnostic_msgs::msg::DiagnosticStatus::OK, "BCDU", "Entering DISCHARGE");
  RCLCPP_INFO(this->get_logger(), "[BCDU] Discharging batteries to MBSU...");

  for (const auto& [id, client] : discharge_clients_) {
    if (battery_state_[id].voltage > 70.0f && client->service_is_ready()) {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      client->async_send_request(req);
    }
  }
}

void BcduNode::publishToMbsu(double volts)
{
  for (const auto& [channel_id, pub] : channel_voltage_pubs_) {
    std_msgs::msg::Float64 v;
    v.data = volts;
    pub->publish(v);
  }
}


void BcduNode::publishDiag(uint8_t level, const std::string &name, const std::string &msg)
{
  diagnostic_msgs::msg::DiagnosticStatus d;
  d.level = level;
  d.name = name;
  d.message = msg;
  diag_pub_->publish(d);
}

void BcduNode::publishStatus(const std::string &mode, bool fault, const std::string &fault_msg)
{
  space_station_interfaces::msg::BCDUStatus st;
  st.header.stamp = this->now();
  st.mode = mode;
  st.fault = fault;
  st.fault_message = fault_msg;
  st.bus_voltage = (mode == "charge") ? ssu_nominal_v_ : regulation_voltage_;
  st.regulation_voltage = regulation_voltage_;
  st.current_draw = (mode == "discharge") ? max_discharge_current_A_ : (mode == "charge" ? -max_charge_current_A_ : 0.0);
  status_pub_->publish(st);
}

}  // namespace space_station_eps

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<space_station_eps::BcduNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
