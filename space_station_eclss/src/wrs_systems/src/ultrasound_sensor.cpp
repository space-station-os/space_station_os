#include "space_station_eclss/ultrasound_sensor.h"

UltrasoundSensorPublisher::UltrasoundSensorPublisher() : Node("ultrasound_sensor_publisher") {
    ultrasound_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("/urine_sensor", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     std::bind(&UltrasoundSensorPublisher::publish_sensor_data, this));

    RCLCPP_INFO(this->get_logger(), "Ultrasound Sensor Publisher Initialized...");
}

void UltrasoundSensorPublisher::publish_sensor_data() {
    auto message = sensor_msgs::msg::Range();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "ultrasound_sensor";

    message.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    message.field_of_view = 0.5;  // Approximate field of view
    message.min_range = 0.1;      // 1 cm
    message.max_range = 2.0;       // 2 meters

    // Simulate urine detection: Random range value between 0.05 - 0.3 meters
    message.range = 0.05 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (0.8 - 0.05)));

    RCLCPP_INFO(this->get_logger(), "Publishing Ultrasound Data: %.2f meters", message.range);

    ultrasound_publisher_->publish(message);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UltrasoundSensorPublisher>());
    rclcpp::shutdown();
    return 0;
}
