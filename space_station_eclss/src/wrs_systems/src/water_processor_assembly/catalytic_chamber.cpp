#include "space_station_eclss/water_processor_assembly/catalytic_chamber.h"

MultiFiltrationProcessor::MultiFiltrationProcessor() : Node("wpa_multi_filtration_server"), unprocessed_water_(0.0) {
    multi_filtration_service_ = this->create_service<space_station_eclss::srv::Filteration>(
        "/wpa/filtered_water",
        std::bind(&MultiFiltrationProcessor::handle_multi_filtration, this, std::placeholders::_1, std::placeholders::_2)
    );

    waste_status_publisher_ = this->create_publisher<space_station_eclss::msg::WaterCrew>("/wpa/catalytic_chamber", 10);
    catalytic_reactor_client_ = this->create_client<space_station_eclss::srv::IonBed>("/wpa/catalytic_reactor");

    RCLCPP_INFO(this->get_logger(), "[INIT] Multi-Filtration Server Ready.");
}

void MultiFiltrationProcessor::publish_total_water(double water, double gas_bubbles, double contaminants, double iodine_level, double pressure, double temperature) {
    space_station_eclss::msg::WaterCrew msg;
    msg.water = water;
    msg.gas_bubbles = gas_bubbles;
    msg.contaminants = contaminants;
    msg.iodine_level = iodine_level;
    msg.pressure = pressure;
    msg.temperature = temperature;
    waste_status_publisher_->publish(msg);
}

void MultiFiltrationProcessor::handle_multi_filtration(
    const std::shared_ptr<space_station_eclss::srv::Filteration::Request> request,
    std::shared_ptr<space_station_eclss::srv::Filteration::Response> response) 
{
    double filtered_water = request->filtered_water;
    double contaminants = request->contaminants;
    double organics = request->organics;
    double ammonia = request->ammonia;

    if (filtered_water <= 0) {
        response->message = "Invalid water volume!";
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "[ERROR] %s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[RECEIVED] %.2f L filtered water | Contaminants: %.2f%% | Organics: %.2f%% | Ammonia: %.2f%%",
                filtered_water, contaminants, organics, ammonia);

    remove_ammonia_and_organics(filtered_water, contaminants, organics, ammonia);

    response->success = true;
    response->message = "Multi-Filtration complete. Sending to Catalytic Reactor.";

    publish_total_water(filtered_water, 0.02, contaminants, 0.0, 101.3, 23.5);

    send_to_catalytic_reactor(filtered_water, contaminants, organics);
}

void MultiFiltrationProcessor::remove_ammonia_and_organics(double &filtered_water, double &contaminants, double &organics, double &ammonia) {
    organics *= 0.50;
    ammonia *= 0.60;
    contaminants *= 0.70;

    RCLCPP_INFO(this->get_logger(), "[FILTERED] Organics: %.2f%% | Ammonia: %.2f%% | Contaminants: %.2f%%",
                organics, ammonia, contaminants);
}

void MultiFiltrationProcessor::send_to_catalytic_reactor(double filtered_water, double remaining_contaminants, double organics) {
    if (!catalytic_reactor_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_FATAL(this->get_logger(), "[FAIL] Catalytic Reactor Service Unavailable. Holding water.");
        unprocessed_water_ = filtered_water;
        return;
    }

    auto request = std::make_shared<space_station_eclss::srv::IonBed::Request>();
    request->filtered_water = filtered_water;
    request->contaminants = remaining_contaminants;

    RCLCPP_INFO(this->get_logger(), "[SEND] %.2f L → Catalytic Reactor | Contaminants: %.2f%%", filtered_water, remaining_contaminants);

    auto future_result = catalytic_reactor_client_->async_send_request(request,
        std::bind(&MultiFiltrationProcessor::handle_catalytic_reactor_response, this, std::placeholders::_1));
}

void MultiFiltrationProcessor::handle_catalytic_reactor_response(rclcpp::Client<space_station_eclss::srv::IonBed>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "[SUCCESS] Catalytic Reactor accepted water: %s", response->message.c_str());
            unprocessed_water_ = 0.0;
        } else {
            RCLCPP_WARN(this->get_logger(), "[REJECTED] Catalytic Reactor: %s. Holding water.", response->message.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "[EXCEPTION] Reactor call failed: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiFiltrationProcessor>());
    rclcpp::shutdown();
    return 0;
}
