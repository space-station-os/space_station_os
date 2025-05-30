#ifndef STL_MONITOR_HPP_
#define STL_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <space_station_eclss/msg/air_data.hpp>

#include <map>
#include <string>
#include <utility>
#include <chrono>

class STLMonitor : public rclcpp::Node
{
public:
    STLMonitor();
    ~STLMonitor();

private:
    // Bounds for STL specifications
    std::map<std::string, std::pair<double, double>> bounds_;

    // Consecutive out-of-bound counts for escalation
    std::map<std::string, int> violation_counters_;

    // Consecutive in-bound counts for recovery
    std::map<std::string, int> recovery_counters_;

    // Current status: PASS, FAIL, CODE_RED
    std::map<std::string, std::string> system_state_;

    // Timestamp of the last update
    rclcpp::Time last_update_time_;

    // ROS Subscribers
    rclcpp::Subscription<space_station_eclss::msg::AirData>::SharedPtr collector_sub_;
    rclcpp::Subscription<space_station_eclss::msg::AirData>::SharedPtr desiccant_sub_;
    rclcpp::Subscription<space_station_eclss::msg::AirData>::SharedPtr adsorbent_sub_;

    // ROS Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gui_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr crew_alert_pub_;

    // Helpers
    bool checkBounds(double value, const std::pair<double, double>& bounds);
    double robustness(double value, const std::pair<double, double>& bounds);

    void checkSignal(const std::string& label, const std::string& key, double value, const std::pair<double, double>& bounds);
    void publishCrewAlert(const std::string& label, double value);

    // Topic callbacks
    void collectorCallback(const space_station_eclss::msg::AirData::SharedPtr msg);
    void desiccantCallback(const space_station_eclss::msg::AirData::SharedPtr msg);
    void adsorbentCallback(const space_station_eclss::msg::AirData::SharedPtr msg);
};

#endif  // STL_MONITOR_HPP_
