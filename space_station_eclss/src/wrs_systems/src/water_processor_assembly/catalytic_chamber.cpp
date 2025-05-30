#include "space_station_eclss/water_processor_assembly/catalytic_chamber.h"

MultiFiltrationProcessor::MultiFiltrationProcessor() : Node("wpa_multi_filtration_server"), unprocessed_water_(0.0) {
    multi_filtration_service_ = this->create_service<space_station_eclss::srv::Filteration>(
        "/wpa/filtered_water",
        std::bind(&MultiFiltrationProcessor::handle_multi_filtration, this, std::placeholders::_1, std::placeholders::_2)
    );

    waste_status_publisher_ = this->create_publisher<space_station_eclss::msg::WaterCrew>("/wpa/catalytic_chamber", 10);
    catalytic_reactor_client_ = this->create_client<space_station_eclss::srv::IonBed>("/wpa/catalytic_reactor");

    RCLCPP_INFO(this->get_logger(), "Multi-Filtration Processing Server Ready...");
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
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received %.2f liters of filtered water.", filtered_water);
    RCLCPP_INFO(this->get_logger(), "Initial Contaminants: %.2f%% | Organics: %.2f%% | Ammonia: %.2f%%", contaminants, organics, ammonia);

    remove_ammonia_and_organics(filtered_water, contaminants, organics, ammonia);

    response->success = true;
    response->message = "Multi-Filtration complete! Sending to Catalytic Reactor.";

    // Publish updated water state to dashboard
    publish_total_water(filtered_water, 0.02, contaminants, 0.0, 101.3, 23.5);

    // Send to catalytic reactor using updated contaminants
    send_to_catalytic_reactor(filtered_water, contaminants, organics);
}

void MultiFiltrationProcessor::remove_ammonia_and_organics(double &filtered_water, double &contaminants, double &organics, double &ammonia) {
    organics *= 0.50;
    ammonia *= 0.60;
    contaminants *= 0.70;

    RCLCPP_INFO(this->get_logger(), "Multifiltration Bed: Removed 50%% Organics, 40%% Ammonia.");
    RCLCPP_INFO(this->get_logger(), "Remaining Organics: %.2f%% | Remaining Ammonia: %.2f%%", organics, ammonia);
}

void MultiFiltrationProcessor::send_to_catalytic_reactor(double filtered_water, double remaining_contaminants, double organics) {
    if (!catalytic_reactor_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_FATAL(this->get_logger(), "Catalytic Reactor Service Unavailable! Retaining processed water.");
        unprocessed_water_ = filtered_water;
        return;
    }

    auto request = std::make_shared<space_station_eclss::srv::IonBed::Request>();
    request->filtered_water = filtered_water;
    request->contaminants = remaining_contaminants;

    RCLCPP_INFO(this->get_logger(), "Sending %.2f L filtered water with %.2f%% contaminants...",
                filtered_water, remaining_contaminants);

    auto future_result = catalytic_reactor_client_->async_send_request(request,
        std::bind(&MultiFiltrationProcessor::handle_catalytic_reactor_response, this, std::placeholders::_1));
}

void MultiFiltrationProcessor::handle_catalytic_reactor_response(rclcpp::Client<space_station_eclss::srv::IonBed>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Catalytic Reactor successfully processed water: %s", response->message.c_str());
            unprocessed_water_ = 0.0;
        } else {
            RCLCPP_WARN(this->get_logger(), "Catalytic Reactor failed: %s. Retaining processed water.", response->message.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while calling Catalytic Reactor service: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiFiltrationProcessor>());
    rclcpp::shutdown();
    return 0;
}
