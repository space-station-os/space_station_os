#include "space_station_eclss/water_processor_assembly/filteration_unit.h"

WaterProcessorAssembly::WaterProcessorAssembly() : Node("wpa_service_server") {
    wpa_service_ = this->create_service<space_station_eclss::srv::Distillation>(
        "/wpa/process_water",
        std::bind(&WaterProcessorAssembly::handle_water_processing, this, std::placeholders::_1, std::placeholders::_2)
    );

    waste_status_publisher_ = this->create_publisher<space_station_eclss::msg::WaterCrew>("/wpa/filteration_status", 10);
    next_stage_client_ = this->create_client<space_station_eclss::srv::Filteration>("/wpa/filtered_water");

    RCLCPP_INFO(this->get_logger(), "Water Processor Assembly Server Ready...");
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
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    double initial_organics = 30.0;
    double initial_ammonia = 50.0;

    RCLCPP_INFO(this->get_logger(), "Received %.2f liters of urine water with %.2f%% contaminants.", urine_volume, contaminants);
    RCLCPP_INFO(this->get_logger(), "Initial Organic Load: %.2f%% | Initial Ammonia Load: %.2f%%", initial_organics, initial_ammonia);

    double contaminants_after_efa = contaminants * 0.75;
    initial_organics *= 0.75;
    initial_ammonia *= 0.85;

    RCLCPP_INFO(this->get_logger(), "EFA Filter: Removed 25%% contaminants. Remaining: %.2f%%", contaminants_after_efa);
    RCLCPP_INFO(this->get_logger(), "EFA Filter: Organics Remaining: %.2f%% | Ammonia Remaining: %.2f%%", initial_organics, initial_ammonia);

    double contaminants_after_particulate = contaminants_after_efa * 0.40;
    initial_organics *= 0.60;
    initial_ammonia *= 0.75;

    RCLCPP_INFO(this->get_logger(), "Particulate Filter: Removed 60%% contaminants. Remaining: %.2f%%", contaminants_after_particulate);
    RCLCPP_INFO(this->get_logger(), "Particulate Filter: Organics Remaining: %.2f%% | Ammonia Remaining: %.2f%%", initial_organics, initial_ammonia);

    
    publish_status(urine_volume, contaminants_after_particulate, initial_organics, initial_ammonia);

    response->success = true;
    response->message = "Received distilled water. Onto next stage of filtration...";

    send_to_next_stage(urine_volume, contaminants_after_particulate, initial_organics, initial_ammonia);
}

void WaterProcessorAssembly::send_to_next_stage(double filtered_water, double remaining_contaminants, double organics, double ammonia) {
    if (!next_stage_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_FATAL(this->get_logger(), "Next stage filtration service unavailable! Retaining processed water.");
        return;
    }

    auto request = std::make_shared<space_station_eclss::srv::Filteration::Request>();
    request->filtered_water = filtered_water;
    request->contaminants = remaining_contaminants;
    request->organics = organics;
    request->ammonia = ammonia;

    RCLCPP_INFO(this->get_logger(), "Sending %.2f L filtered water with %.2f%% contaminants, %.2f%% organics, %.2f%% ammonia to next stage...",
                filtered_water, remaining_contaminants, organics, ammonia);

    auto future_result = next_stage_client_->async_send_request(request,
        std::bind(&WaterProcessorAssembly::handle_next_stage_response, this, std::placeholders::_1));
}

void WaterProcessorAssembly::handle_next_stage_response(rclcpp::Client<space_station_eclss::srv::Filteration>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Next Stage Filtration successfully completed: %s", response->message.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Next Stage Filtration failed: %s", response->message.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while calling Next Stage Filtration service: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaterProcessorAssembly>());
    rclcpp::shutdown();
    return 0;
}
