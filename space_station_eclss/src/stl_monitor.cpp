#include "space_station_eclss/stl_monitor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <sstream>
#include <unordered_map>
#include <iomanip>

using namespace std::chrono_literals;

STLMonitor::STLMonitor() : Node("stl_monitor")
{
    bounds_["collector"] = {0.0, 98.89};
    bounds_["desiccant_moisture"] = {0.0, 153.21};
    bounds_["desiccant_contaminants"] = {0.0, 35.08};
    bounds_["adsorbent"] = {0.0, 114.51};

    violation_counters_ = {
        {"collector", 0}, {"desiccant_moisture", 0}, {"desiccant_contaminants", 0}, {"adsorbent", 0}
    };

    recovery_counters_ = {
        {"collector", 0}, {"desiccant_moisture", 0}, {"desiccant_contaminants", 0}, {"adsorbent", 0}
    };

    system_state_ = {
        {"collector", "OK"}, {"desiccant_moisture", "OK"},
        {"desiccant_contaminants", "OK"}, {"adsorbent", "OK"}
    };

    last_update_time_ = this->now();

    collector_sub_ = this->create_subscription<space_station_eclss::msg::AirData>(
        "/collector_air_quality", 10,
        std::bind(&STLMonitor::collectorCallback, this, std::placeholders::_1));

    desiccant_sub_ = this->create_subscription<space_station_eclss::msg::AirData>(
        "/desiccant_air_quality", 10,
        std::bind(&STLMonitor::desiccantCallback, this, std::placeholders::_1));

    adsorbent_sub_ = this->create_subscription<space_station_eclss::msg::AirData>(
        "/adsorbent_air_quality", 10,
        std::bind(&STLMonitor::adsorbentCallback, this, std::placeholders::_1));

    gui_pub_ = this->create_publisher<std_msgs::msg::String>("/stl_monitor/status", 10);
    crew_alert_pub_ = this->create_publisher<std_msgs::msg::String>("/crew_alert", 10);

    RCLCPP_INFO(this->get_logger(), "STL Monitor Node Initialized with Robustness Tracking and Alerts.");
}

STLMonitor::~STLMonitor() {}

bool STLMonitor::checkBounds(double value, const std::pair<double, double>& bounds)
{
    return value >= bounds.first && value <= bounds.second;
}

double STLMonitor::robustness(double value, const std::pair<double, double>& bounds)
{
    return std::min(value - bounds.first, bounds.second - value);
}

void STLMonitor::checkSignal(const std::string& label, const std::string& key, double value, const std::pair<double, double>& bounds)
{
    double r = robustness(value, bounds);
    last_update_time_ = this->now();

    bool in_bounds = checkBounds(value, bounds);
    std::string new_status;

    if (in_bounds) {
        violation_counters_[key] = 0;

        if (system_state_[key] != "OK") {
            recovery_counters_[key]++;
            if (recovery_counters_[key] >= 5) {
                new_status = "OK";
                RCLCPP_INFO(this->get_logger(), "[%s] Fully recovered. Status: OK. Value: %.2f", key.c_str(), value);
            } else {
                new_status = system_state_[key];
                RCLCPP_INFO(this->get_logger(), "[%s] Recovering... (%d/5) Value: %.2f", key.c_str(), recovery_counters_[key], value);
            }
        } else {
            new_status = "OK";
            RCLCPP_INFO(this->get_logger(), "[%s] OK. Value: %.2f | Robustness: %.2f", key.c_str(), value, r);
        }
    } else {
        recovery_counters_[key] = 0;
        violation_counters_[key]++;

        if (violation_counters_[key] > 6) {
            new_status = "CODE_RED";
            if (system_state_[key] != "CODE_RED") {
                publishCrewAlert(label, value);
                RCLCPP_FATAL(this->get_logger(), "[%s] CRITICAL VIOLATION — CODE RED. Value: %.2f", key.c_str(), value);
            }
        } else {
            new_status = "FAIL";
            RCLCPP_WARN(this->get_logger(), "[%s] OUT OF BOUNDS (%d). Value: %.2f", key.c_str(), violation_counters_[key], value);
        }
    }

    system_state_[key] = new_status;

    // Publish current status map to GUI
    std::ostringstream json;
    json << "{";
    bool first = true;
    for (const auto& kv : system_state_) {
        if (!first) json << ", ";
        json << "\"" << kv.first << "\": \"" << kv.second << "\"";
        first = false;
    }
    json << "}";

    std_msgs::msg::String msg;
    msg.data = json.str();
    gui_pub_->publish(msg);
}

void STLMonitor::publishCrewAlert(const std::string& label, double value)
{
    std_msgs::msg::String alert;
    alert.data = "[ALERT] " + label + " triggered CODE RED. Immediate action required! Value: " + std::to_string(value);
    crew_alert_pub_->publish(alert);
    RCLCPP_FATAL(this->get_logger(), "%s", alert.data.c_str());
}

void STLMonitor::collectorCallback(const space_station_eclss::msg::AirData::SharedPtr msg)
{
    checkSignal("COLLECTOR (CO2)", "collector", msg->co2_mass, bounds_["collector"]);
}

void STLMonitor::desiccantCallback(const space_station_eclss::msg::AirData::SharedPtr msg)
{
    checkSignal("DESICCANT (Moisture)", "desiccant_moisture", msg->moisture_content, bounds_["desiccant_moisture"]);
    checkSignal("DESICCANT (Contaminants)", "desiccant_contaminants", msg->contaminants, bounds_["desiccant_contaminants"]);
}

void STLMonitor::adsorbentCallback(const space_station_eclss::msg::AirData::SharedPtr msg)
{
    checkSignal("ADSORBENT (CO2)", "adsorbent", msg->co2_mass, bounds_["adsorbent"]);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<STLMonitor>());
    rclcpp::shutdown();
    return 0;
}
