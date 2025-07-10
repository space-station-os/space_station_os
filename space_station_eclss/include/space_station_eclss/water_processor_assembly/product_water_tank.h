#ifndef PRODUCT_WATER_TANK_HPP
#define PRODUCT_WATER_TANK_HPP

#include <rclcpp/rclcpp.hpp>
#include "space_station_eclss/srv/water.hpp"
#include "space_station_eclss/srv/clean_water.hpp"
#include "space_station_eclss/msg/water_crew.hpp"

class ProductWaterTank : public rclcpp::Node {
public:
    ProductWaterTank();

private:
    rclcpp::Service<space_station_eclss::srv::Water>::SharedPtr water_tank_service_;
    rclcpp::Service<space_station_eclss::srv::CleanWater>::SharedPtr dispense_water_service_;
    rclcpp::Publisher<space_station_eclss::msg::WaterCrew>::SharedPtr tank_status_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    double current_water_volume_;
    double current_gas_bubbles_;
    double current_contaminants_;
    double current_iodine_level_;
    double current_pressure_;
    double current_temperature_;

    void handle_water_tank(
        const std::shared_ptr<space_station_eclss::srv::Water::Request> request,
        std::shared_ptr<space_station_eclss::srv::Water::Response> response);

    void handle_dispense_water(
        const std::shared_ptr<space_station_eclss::srv::CleanWater::Request> request,
        std::shared_ptr<space_station_eclss::srv::CleanWater::Response> response);

    void publish_tank_status();
};

#endif // PRODUCT_WATER_TANK_HPP
