#include "space_station_eclss/water_processor_assembly/ionization_bed.h"

CatalyticReactorProcessor::CatalyticReactorProcessor() : Node("wpa_catalytic_reactor_server"), unprocessed_water_(0.0) {
    catalytic_reactor_service_ = this->create_service<space_station_eclss::srv::IonBed>(
        "/wpa/catalytic_reactor",
        std::bind(&CatalyticReactorProcessor::handle_catalytic_reactor, this, std::placeholders::_1, std::placeholders::_2)
    );

    waste_status_publisher_ = this->create_publisher<space_station_eclss::msg::WaterCrew>("/wpa/ionization_status", 10);
    product_water_tank_client_ = this->create_client<space_station_eclss::srv::Water>("/wpa/product_water_tank");

    RCLCPP_INFO(this->get_logger(), "Catalytic Reactor Processing Server Ready...");
}

void CatalyticReactorProcessor::publish_total_water(double filtered_water, double contaminants, double iodine_level, double gas_bubbles) {
    space_station_eclss::msg::WaterCrew msg;
    msg.water = filtered_water;
    msg.gas_bubbles = gas_bubbles;
    msg.contaminants = contaminants;
    msg.iodine_level = iodine_level;
    msg.pressure = 101.3;
    msg.temperature = 23.5;
    waste_status_publisher_->publish(msg);
}

void CatalyticReactorProcessor::handle_catalytic_reactor(
    const std::shared_ptr<space_station_eclss::srv::IonBed::Request> request,
    std::shared_ptr<space_station_eclss::srv::IonBed::Response> response) 
{
    double filtered_water = request->filtered_water;
    double contaminants = request->contaminants;

    if (filtered_water <= 0) {
        response->message = "Invalid water volume!";
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received %.2f liters of filtered water.", filtered_water);
    RCLCPP_INFO(this->get_logger(), "Initial Contaminants: %.2f%%", contaminants);

    double iodine_level = 0.0;
    double gas_bubbles = 0.05;

    process_ion_exchange(filtered_water, contaminants, iodine_level, gas_bubbles);

    publish_total_water(filtered_water, contaminants, iodine_level, gas_bubbles);

    response->success = true;
    response->message = "Ion Exchange Complete! Sending to Product Water Tank.";

    send_to_product_water_tank(filtered_water, gas_bubbles, contaminants, iodine_level, 101.3, 25.0);
}

void CatalyticReactorProcessor::process_ion_exchange(double &filtered_water, double &contaminants, double &iodine_level, double &gas_bubbles) {
    contaminants *= 0.2;
    iodine_level = filtered_water * 0.05;
    gas_bubbles *= 0.5;

    RCLCPP_INFO(this->get_logger(), "Ion Exchange Bed: Removed 80%% Contaminants.");
    RCLCPP_INFO(this->get_logger(), "Final Contaminants: %.2f%% | Iodine Level: %.2f mg/L | Gas Bubbles: %.2f%%", 
                contaminants, iodine_level, gas_bubbles);
}

void CatalyticReactorProcessor::send_to_product_water_tank(double water, double gas_bubbles, double contaminants, double iodine_level, double pressure, double temperature) {
    if (!product_water_tank_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_FATAL(this->get_logger(), "Product Water Tank Service Unavailable! Retaining processed water.");
        unprocessed_water_ = water;
        return;
    }

    auto request = std::make_shared<space_station_eclss::srv::Water::Request>();
    request->water = water;
    request->gas_bubbles = gas_bubbles;
    request->contaminants = contaminants;
    request->iodine_level = iodine_level;
    request->pressure = pressure;
    request->temperature = temperature;

    RCLCPP_INFO(this->get_logger(), "Sending %.2f L purified water with %.2f%% contaminants, %.2f mg/L iodine, %.2f%% gas bubbles to Product Water Tank...",
                water, contaminants, iodine_level, gas_bubbles);

    auto future_result = product_water_tank_client_->async_send_request(request,
        std::bind(&CatalyticReactorProcessor::handle_product_water_tank_response, this, std::placeholders::_1));
}

void CatalyticReactorProcessor::handle_product_water_tank_response(rclcpp::Client<space_station_eclss::srv::Water>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Product Water Tank successfully received purified water: %s", response->message.c_str());
            unprocessed_water_ = 0.0;
        } else {
            RCLCPP_WARN(this->get_logger(), "Product Water Tank rejected water: %s. Retaining processed water.", response->message.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while calling Product Water Tank service: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CatalyticReactorProcessor>());
    rclcpp::shutdown();
    return 0;
}
