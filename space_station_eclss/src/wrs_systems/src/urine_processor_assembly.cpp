#include "space_station_eclss/urine_processor_assembly.h"

UrineProcessorAssembly::UrineProcessorAssembly() : Node("upa_service_server") {
    upa_service_ = this->create_service<space_station_eclss::srv::Upa>(
        "/upa/process_urine",
        std::bind(&UrineProcessorAssembly::handle_urine_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    waste_status_pub_ = this->create_publisher<space_station_eclss::msg::WaterCrew>("/whc/upa", 10);
    wpa_client_ = this->create_client<space_station_eclss::srv::Distillation>("/wpa/process_water");

    RCLCPP_INFO(this->get_logger(), "Urine Processor Assembly Server Ready...");
}

void UrineProcessorAssembly::publish_status(double water, double contaminants, double temperature) {
    space_station_eclss::msg::WaterCrew msg;
    msg.water = water;
    msg.gas_bubbles = 0.01 * water;  // Simulated
    msg.contaminants = contaminants;
    msg.iodine_level = 0.005;        // Placeholder
    msg.pressure = 101.3;
    msg.temperature = temperature;
    waste_status_pub_->publish(msg);
}

void UrineProcessorAssembly::handle_urine_request(
    const std::shared_ptr<space_station_eclss::srv::Upa::Request> request,
    std::shared_ptr<space_station_eclss::srv::Upa::Response> response) 
{
    double urine_volume = request->urine;
    if (urine_volume <= 0) {
        response->message = "Invalid urine volume!";
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received %.2f liters of urine from Waste Collection Tank...", urine_volume);

    double distilled_urine = 0.0;
    double contaminants = 0.0;

    simulate_distillation(urine_volume, distilled_urine, contaminants);
    simulate_purge_pump(distilled_urine);

    response->success = true;
    response->message = "Distillation complete! Sending to Purge Pump.";

    send_to_wpa(distilled_urine, contaminants);
}

void UrineProcessorAssembly::simulate_distillation(double urine_volume, double &distilled_urine, double &contaminants) {
    RCLCPP_INFO(this->get_logger(), "Starting Distillation Process...");

    double batch_size = urine_volume / 2.0;
    for (int i = 0; i < 2; i++) {
        double temp_contaminants = batch_size * 0.1;
        double temp_distilled = batch_size - temp_contaminants;

        RCLCPP_INFO(this->get_logger(), "Batch %d: Heated to 100°C - %.2f L of clean water extracted.", i + 1, temp_distilled);
        rclcpp::sleep_for(std::chrono::seconds(2));

        contaminants += temp_contaminants;
        distilled_urine += temp_distilled;
    }

    RCLCPP_INFO(this->get_logger(), "Distillation Complete! Total clean water: %.2f L, Contaminants: %.2f L", distilled_urine, contaminants);
}

void UrineProcessorAssembly::simulate_purge_pump(double &distilled_urine) {
    RCLCPP_INFO(this->get_logger(), "Purge Pump Activated: Cooling down water...");

    double temperature = 100.0;

    for (int i = 0; i < 5; i++) {
        temperature -= 20.0;
        RCLCPP_INFO(this->get_logger(), "Cooling... Current Temperature: %.2f°C", temperature);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    publish_status(distilled_urine, 0.0, 20.0);

    RCLCPP_INFO(this->get_logger(), "Condensation complete! %.2f L clean water is ready for WPA.", distilled_urine);
}

void UrineProcessorAssembly::send_to_wpa(double processed_water, double contaminants) {
    if (!wpa_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Water Processor Assembly (WPA) Service Unavailable!");
        return;
    }

    auto request = std::make_shared<space_station_eclss::srv::Distillation::Request>();
    request->urine = processed_water;
    request->contaminants = contaminants;

    RCLCPP_INFO(this->get_logger(), "Sending %.2f L clean water and %.2f L contaminants to WPA...", processed_water, contaminants);

    auto future_result = wpa_client_->async_send_request(request,
        std::bind(&UrineProcessorAssembly::handle_wpa_response, this, std::placeholders::_1));
}

void UrineProcessorAssembly::handle_wpa_response(rclcpp::Client<space_station_eclss::srv::Distillation>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "WPA processed water successfully: %s", response->message.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "WPA failed to process water: %s", response->message.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while calling WPA service: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UrineProcessorAssembly>());
    rclcpp::shutdown();
    return 0;
}
