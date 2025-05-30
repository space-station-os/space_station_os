#include "space_station_eclss/deionization_bed.h"
#include <chrono>

using namespace std::chrono_literals;

IonizationBed::IonizationBed()
    : Node("ionization_bed"),
      is_active(false),
      water_(0.0),
      contaminants_(0.0),
      gas_bubbles_(0.0),
      iodine_level_(0.0) {

    // Start the /deionization_chamber server to receive water
    deionization_server_ = this->create_service<space_station_eclss::srv::Water>(
        "/deionization_chamber",
        std::bind(&IonizationBed::handle_deionization_request, this, std::placeholders::_1, std::placeholders::_2));

    // Client to send processed water to /electrolysis
    electrolysis_client_ = this->create_client<space_station_eclss::srv::Water>("/electrolysis");

    RCLCPP_INFO(this->get_logger(), "Ionization Bed Node Initialized.");
}

void IonizationBed::handle_deionization_request(
    const std::shared_ptr<space_station_eclss::srv::Water::Request> request,
    std::shared_ptr<space_station_eclss::srv::Water::Response> response) {
    
    if (is_active) {
        response->success = false;
        response->message = "Ionization bed is already processing.";
        RCLCPP_WARN(this->get_logger(), "Ionization bed is already active.");
        return;
    }

    // Store received water parameters
    water_ = request->water;
    contaminants_ = request->contaminants;
    gas_bubbles_ = request->gas_bubbles;
    iodine_level_ = request->iodine_level;

    RCLCPP_INFO(this->get_logger(), "Received water: %.2f L, Contaminants: %.2f ppm, Iodine: %.2f ppm",
                water_, contaminants_, iodine_level_);

    // Activate processing
    is_active = true;
    processing_timer_ = this->create_wall_timer(
        1s, std::bind(&IonizationBed::contamination_removal_pipeline, this));

    response->success = true;
    response->message = "Ionization bed processing started.";
}

void IonizationBed::contamination_removal_pipeline() {
    if (water_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "No water available to process.");
        is_active = false;
        processing_timer_->cancel();
        return;
    }

    deionization();
    contamination_removal();
    gas_sensor();

    if (iodine_level_ == 0.0 && contaminants_ == 0.0 && gas_bubbles_ == 0.0) {
        processing_timer_->cancel();
        send_to_electrolysis();
        is_active = false;
    }
}

void IonizationBed::deionization() {
    if (water_ >= 10.0) {
        double iodine_removal_rate = 0.5;  // Slower removal for realistic simulation
        iodine_level_ -= iodine_removal_rate;
        if (iodine_level_ < 0.01) iodine_level_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "Deionization in progress. Remaining iodine: %.2f", iodine_level_);
    }
}

void IonizationBed::contamination_removal() {
    double contaminants_decrement = 1.5;  // Slow reduction rate
    contaminants_ -= contaminants_decrement;
    if (contaminants_ < 0.0) contaminants_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "Contamination removal in progress. Remaining contaminants: %.2f", contaminants_);
}

void IonizationBed::gas_sensor() {
    if (gas_bubbles_ > 0.0) {
        RCLCPP_WARN(this->get_logger(), "Gas bubbles detected: %.2f. Redirecting via three-way valve.", gas_bubbles_);
        open_three_way_valve();
    } else {
        RCLCPP_INFO(this->get_logger(), "No gas bubbles detected.");
        RCLCPP_INFO(this->get_logger(), "===========================");
    }
}

void IonizationBed::open_three_way_valve() {
    RCLCPP_INFO(this->get_logger(), "Three-way valve opened. Redirecting water with gas bubbles.");
    gas_bubbles_ = 0.0;
}

void IonizationBed::send_to_electrolysis() {
    if (!electrolysis_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(this->get_logger(), "Electrolysis service is not available!");
        return;
    }

    auto request = std::make_shared<space_station_eclss::srv::Water::Request>();
    request->water = water_;
    request->contaminants = contaminants_;
    request->iodine_level = iodine_level_;
    request->gas_bubbles = gas_bubbles_;
    request->pressure = 14.0;
    request->temperature = 25.0;

    RCLCPP_INFO(this->get_logger(), "Sending processed water to electrolysis: %.2f L", water_);

    auto future = electrolysis_client_->async_send_request(request,
        [this](rclcpp::Client<space_station_eclss::srv::Water>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Water successfully transferred to electrolysis.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to send water to electrolysis: %s", response->message.c_str());
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Exception while calling electrolysis service: %s", e.what());
            }

            // Reset active status to allow new deionization requests
            is_active = false;
            RCLCPP_INFO(this->get_logger(), "Ionization bed is now available for new requests.");
        }
    );

    // Reset water after sending to electrolysis
    water_ = 0.0;
    contaminants_ = 0.0;
    iodine_level_ = 0.0;
    gas_bubbles_ = 0.0;
}


// Main Function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IonizationBed>());
    rclcpp::shutdown();
    return 0;
}
