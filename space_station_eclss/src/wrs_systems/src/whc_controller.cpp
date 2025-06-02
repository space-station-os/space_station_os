#include "space_station_eclss/whc_controller.h"
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float64.hpp>
#include "space_station_eclss/msg/water_crew.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

WasteHygieneCompartment::WasteHygieneCompartment() 
    : Node("whc_controller"),
      urination_detected(false),
      urine_volume(0.3),
      pretreatment_volume(0.05),
      flush_volume(1.0),
      total_water_volume(0.0) {
    
    ultrasound_subscriber_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/urine_sensor", 10, std::bind(&WasteHygieneCompartment::urine_callback, this, _1)
    );

    water_volume_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/whc/water_volume", 10);
    waste_status_publisher_ = this->create_publisher<space_station_eclss::msg::WaterCrew>("/whc_controller_water", 10);

    RCLCPP_INFO(this->get_logger(), "[INIT] WHC Controller Node Initialized");
}

void WasteHygieneCompartment::urine_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
    static rclcpp::Time last_urine_time = this->now();

    if (msg->range < 0.6) {
        if (!urination_detected) {
            urination_detected = true;
            last_urine_time = this->now();

            timer_ = this->create_wall_timer(
                2s, std::bind(&WasteHygieneCompartment::complete_urination_cycle, this));
            
            RCLCPP_INFO(this->get_logger(), "[DETECTED] Urination started.");
        }
        return;
    }

    if (urination_detected && (this->now() - last_urine_time).seconds() > 2.0) {
        urination_detected = false;
    }
}

void WasteHygieneCompartment::complete_urination_cycle() {
    RCLCPP_INFO(this->get_logger(), "[CYCLE] Urination complete. Adding pretreatment and flushing.");

    add_pretreatment();
    flush();
    publish_total_water();

    if (timer_) timer_->cancel();
    urination_detected = false;
}

void WasteHygieneCompartment::add_pretreatment() {
    total_water_volume += pretreatment_volume;
}

void WasteHygieneCompartment::flush() {
    total_water_volume += flush_volume;
}

void WasteHygieneCompartment::publish_total_water() {
    total_water_volume += urine_volume;

    std_msgs::msg::Float64 msg;
    msg.data = total_water_volume;
    water_volume_publisher_->publish(msg);

    space_station_eclss::msg::WaterCrew status;
    status.water = total_water_volume;
    status.gas_bubbles = 0.01 * total_water_volume;
    status.contaminants = 0.8 * total_water_volume;
    status.iodine_level = 0.02;
    status.pressure = 101.3;
    status.temperature = 22.0;
    waste_status_publisher_->publish(status);

    RCLCPP_INFO(this->get_logger(), "[PUBLISH] Total WHC water volume: %.2f L", total_water_volume);

    total_water_volume = 0.0;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WasteHygieneCompartment>());
    rclcpp::shutdown();
    return 0;
}
