#ifndef ULTRASOUND_SENSOR_PUB_HPP
#define ULTRASOUND_SENSOR_PUB_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/range.hpp"

class UltrasoundSensorPublisher : public rclcpp::Node {
public:
    UltrasoundSensorPublisher();

private:
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasound_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_sensor_data();
};

#endif // ULTRASOUND_SENSOR_PUB_HPP
