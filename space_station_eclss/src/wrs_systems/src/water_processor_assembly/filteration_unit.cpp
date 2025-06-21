#include "space_station_eclss/water_processor_assembly/filteration_unit.h"

WaterProcessorAssembly::WaterProcessorAssembly() : Node("wpa_service_server") {
    wpa_service_ = this->create_service<space_station_eclss::srv::Distillation>(
        "/wpa/process_water",
        std::bind(&WaterProcessorAssembly::handle_water_processing, this, std::placeholders::_1, std::placeholders::_2)
    );

    waste_status_publisher_ = this->create_publisher<space_station_eclss::msg::WaterCrew>("/wpa/filteration_status", 10);
    next_stage_client_ = this->create_client<space_station_eclss::srv::Filteration>("/wpa/filtered_water");

    RCLCPP_INFO(this->get_logger(), "[INIT] Water Processor Assembly Server Ready.");
}

void WaterProcessorAssembly::publish_status(double water, double contaminants, double organics, double ammonia) {
    space_station_eclss::msg::WaterCrew msg;
    msg.water = water;
    msg.gas_bubbles = 0.0;
    msg.contaminants = contaminants;
    msg.iodine_level = 0.01;
    msg.pressure = 101.3;
    msg.temperature = 22.5;
    waste_status_publisher_->publish(msg);
}

void WaterProcessorAssembly::handle_water_processing(
    const std::shared_ptr<space_station_eclss::srv::Distillation::Request> request,
    std::shared_ptr<space_station_eclss::srv::Distillation::Response> response) 
{
    double urine_volume = request->urine;
    double contaminants = request->contaminants;

    if (urine_volume <= 0) {
        response->message = "Invalid water volume!";
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "[ERROR] %s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[RECEIVED] %.2f L urine water | Contaminants: %.2f%%", urine_volume, contaminants);

    double organics = 30.0;
    double ammonia = 50.0;

    // Stage 1: External Filter Assembly
    contaminants *= 0.75;
    organics *= 0.75;
    ammonia *= 0.85;

    // Stage 2: Particulate Filter
    contaminants *= 0.40;
    organics *= 0.60;
    ammonia *= 0.75;

    RCLCPP_INFO(this->get_logger(), "[FILTERED] Contaminants: %.2f%% | Organics: %.2f%% | Ammonia: %.2f%%",
                contaminants, organics, ammonia);

    publish_status(urine_volume, contaminants, organics, ammonia);

    response->success = true;
    response->message = "Filtered successfully. Passing to next stage.";

    send_to_next_stage(urine_volume, contaminants, organics, ammonia);
}

void WaterProcessorAssembly::send_to_next_stage(double filtered_water, double contaminants, double organics, double ammonia) {
    if (!next_stage_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_FATAL(this->get_logger(), "[FAIL] Next filtration stage unavailable. Holding water.");
        return;
    }

    auto request = std::make_shared<space_station_eclss::srv::Filteration::Request>();
    request->filtered_water = filtered_water;
    request->contaminants = contaminants;
    request->organics = organics;
    request->ammonia = ammonia;

    RCLCPP_INFO(this->get_logger(), "[SEND] %.2f L → Next Stage | Contaminants: %.2f%% | Organics: %.2f%% | Ammonia: %.2f%%",
                filtered_water, contaminants, organics, ammonia);

    auto future_result = next_stage_client_->async_send_request(request,
        std::bind(&WaterProcessorAssembly::handle_next_stage_response, this, std::placeholders::_1));
}

void WaterProcessorAssembly::handle_next_stage_response(rclcpp::Client<space_station_eclss::srv::Filteration>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "[SUCCESS] Next Stage: %s", response->message.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "[REJECTED] Next Stage: %s", response->message.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "[EXCEPTION] Next Stage call: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaterProcessorAssembly>());
    rclcpp::shutdown();
    return 0;
}
