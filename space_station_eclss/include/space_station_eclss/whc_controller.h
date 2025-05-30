#ifndef WHC_CONTROLLER_HPP
#define WHC_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include "space_station_eclss/msg/water_crew.hpp"

using namespace std::chrono_literals;
class WasteHygieneCompartment : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultrasound_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr water_volume_publisher_;
    rclcpp::Publisher<space_station_eclss::msg::WaterCrew>::SharedPtr waste_status_publisher_;
   
    rclcpp::TimerBase::SharedPtr timer_;
    bool urination_detected;
    double urine_volume;         // in liters (approx. 0.3L per urination)
    double pretreatment_volume;  // in liters (~0.05L per cycle)
    double flush_volume;         // in liters (~1.0L per cycle)
    double total_water_volume;   // urine + pretreatment + flush water

public:
    WasteHygieneCompartment();
    void urine_callback(const sensor_msgs::msg::Range::SharedPtr msg);
    void add_pretreatment();
    void flush();
    void publish_total_water();
    void complete_urination_cycle();
};

#endif // WHC_CONTROLLER_HPP
