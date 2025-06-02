#include "space_station_eclss/water_processor_assembly/product_water_tank.h"

ProductWaterTank::ProductWaterTank() : Node("wpa_product_water_tank_server"), current_water_volume_(0.0) {
    water_tank_service_ = this->create_service<space_station_eclss::srv::Water>(
        "/wpa/product_water_tank",
        std::bind(&ProductWaterTank::handle_water_tank, this, std::placeholders::_1, std::placeholders::_2)
    );

    dispense_water_service_ = this->create_service<space_station_eclss::srv::CleanWater>(
        "/wpa/dispense_water",
        std::bind(&ProductWaterTank::handle_dispense_water, this, std::placeholders::_1, std::placeholders::_2)
    );

    tank_status_publisher_ = this->create_publisher<space_station_eclss::msg::WaterCrew>(
        "/wpa/tank_status", 10
    );

    publish_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&ProductWaterTank::publish_tank_status, this)
    );

    RCLCPP_INFO(this->get_logger(), "[INIT] Product Water Tank Server Ready.");
}

void ProductWaterTank::handle_water_tank(
    const std::shared_ptr<space_station_eclss::srv::Water::Request> request,
    std::shared_ptr<space_station_eclss::srv::Water::Response> response) 
{
    double incoming_water = request->water;
    double available_space = 2000 - current_water_volume_;

    if (incoming_water > available_space) {
        response->success = false;
        response->message = "Tank full! Can only accept " + std::to_string(available_space) + "L more.";
        RCLCPP_WARN(this->get_logger(), "[LIMIT] Accepting %.2fL, rejecting %.2fL", available_space, incoming_water - available_space);
        incoming_water = available_space;
    }

    current_water_volume_ += incoming_water;
    current_gas_bubbles_ = request->gas_bubbles;
    current_contaminants_ = request->contaminants;
    current_iodine_level_ = request->iodine_level;
    current_pressure_ = request->pressure;
    current_temperature_ = request->temperature;

    response->success = true;
    response->message = "Water successfully stored.";

    RCLCPP_INFO(this->get_logger(), "[STORE] +%.2fL stored. New total: %.2fL", incoming_water, current_water_volume_);
}

void ProductWaterTank::handle_dispense_water(
    const std::shared_ptr<space_station_eclss::srv::CleanWater::Request> request,
    std::shared_ptr<space_station_eclss::srv::CleanWater::Response> response) 
{
    double requested_water = request->water;

    if (requested_water > current_water_volume_) {
        response->success = false;
        response->message = "Not enough water available.";
        RCLCPP_WARN(this->get_logger(), "[BLOCK] Request %.2fL exceeds available %.2fL", requested_water, current_water_volume_);
        return;
    }

    if (current_iodine_level_ < 0.2) {
        response->success = false;
        response->message = "Iodine level too low for safe consumption.";
        RCLCPP_WARN(this->get_logger(), "[BLOCK] Iodine %.2f mg/L below threshold.", current_iodine_level_);
        return;
    }

    current_water_volume_ -= requested_water;
    response->success = true;
    response->message = "Water dispensed.";

    RCLCPP_INFO(this->get_logger(), "[DISPENSE] %.2fL dispensed. Remaining: %.2fL", requested_water, current_water_volume_);
}

void ProductWaterTank::publish_tank_status() {
    space_station_eclss::msg::WaterCrew msg;
    msg.water = current_water_volume_;
    msg.gas_bubbles = current_gas_bubbles_ * 0.9;
    msg.contaminants = current_contaminants_;
    msg.iodine_level = current_iodine_level_;
    msg.pressure = current_pressure_;
    msg.temperature = current_temperature_;

    tank_status_publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "[STATUS] Tank: %.2fL | Iodine: %.2f mg/L", current_water_volume_, current_iodine_level_);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProductWaterTank>());
    rclcpp::shutdown();
    return 0;
}
