#ifndef WATER_SERVICE_HPP
#define WATER_SERVICE_HPP

#include <rclcpp/rclcpp.hpp>
#include "space_station_eclss/srv/water.hpp"
#include "space_station_eclss/msg/water_crew.hpp"

class WaterService : public rclcpp::Node {
public:
    WaterService();

private:
    // Subscriptions
    rclcpp::Subscription<space_station_eclss::msg::WaterCrew>::SharedPtr tank_status_subscriber_;

    // Services
    rclcpp::Service<space_station_eclss::srv::Water>::SharedPtr water_service_;
    rclcpp::Client<space_station_eclss::srv::Water>::SharedPtr deionization_client_;

    // Timer (optional simulated accumulation)
    rclcpp::TimerBase::SharedPtr accumulation_timer_;

    // Internal water tracking
    double water_level_;
    double contaminants_level_;
    double iodine_level_;
    double gas_bubbles_;
    double pressure_;
    double temperature_;

    // Threshold and tank parameters
    double max_tank_capacity_;
    double accumulation_rate_;
    double threshold_level_;

    // Control flags
    bool ready_for_next_request_;

    // Callbacks and helpers
    void tank_status_callback(const space_station_eclss::msg::WaterCrew::SharedPtr msg);
    void water_accumulation();
    void handle_water_request(
        const std::shared_ptr<space_station_eclss::srv::Water::Request> request,
        std::shared_ptr<space_station_eclss::srv::Water::Response> response);
    void send_deionization_request();
    void process_deionization_response(rclcpp::Client<space_station_eclss::srv::Water>::SharedFuture future);
};

#endif  // WATER_SERVICE_HPP
