#include "space_station_eclss/ors_system/deionization_bed.h"
#include <chrono>

using namespace std::chrono_literals;

IonizationBed::IonizationBed()
    : Node("ionization_bed"),
      is_active(false),
      water_(0.0),
      contaminants_(0.0),
      gas_bubbles_(0.0),
      iodine_level_(0.0) {

    deionization_server_ = this->create_service<space_station_eclss::srv::Water>(
        "/deionization_chamber",
        std::bind(&IonizationBed::handle_deionization_request, this, std::placeholders::_1, std::placeholders::_2));

    electrolysis_client_ = this->create_client<space_station_eclss::srv::Water>("/electrolysis");

    RCLCPP_INFO(this->get_logger(), "[INIT] Ionization Bed Node Ready");
}

void IonizationBed::handle_deionization_request(
    const std::shared_ptr<space_station_eclss::srv::Water::Request> request,
    std::shared_ptr<space_station_eclss::srv::Water::Response> response) {

    if (is_active) {
        response->success = false;
        response->message = "Ionization bed is already processing.";
        RCLCPP_WARN(this->get_logger(), "[BUSY] Ionization bed active. Request rejected.");
        return;
    }

    water_ = request->water;
    contaminants_ = request->contaminants;
    gas_bubbles_ = request->gas_bubbles;
    iodine_level_ = request->iodine_level;

    RCLCPP_INFO(this->get_logger(), "[INPUT] Water = %.2f L | Contaminants = %.2f ppm | Iodine = %.2f ppm", 
                water_, contaminants_, iodine_level_);

    is_active = true;
    processing_timer_ = this->create_wall_timer(
        1s, std::bind(&IonizationBed::contamination_removal_pipeline, this));

    response->success = true;
    response->message = "Ionization bed processing started.";
}

void IonizationBed::contamination_removal_pipeline() {
    if (water_ <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] No water available for processing.");
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
    if (water_ > 0.0 && iodine_level_ > 0.0) {
        // Adjust removal based on water volume (simulate scaled processing)
        double rate = std::min(5.0, iodine_level_);
        iodine_level_ -= rate;
        if (iodine_level_ < 0.01) iodine_level_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "[STEP] Deionization: Removed %.2f ppm | Remaining Iodine = %.2f ppm",
                    rate, iodine_level_);
    }
}

void IonizationBed::contamination_removal() {
    if (contaminants_ > 0.0) {
        double rate = std::min(10.0, contaminants_);
        contaminants_ -= rate;
        if (contaminants_ < 0.0) contaminants_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "[STEP] Contaminants Removed: %.2f ppm | Remaining = %.2f ppm",
                    rate, contaminants_);
    }
}


void IonizationBed::gas_sensor() {
    if (gas_bubbles_ > 0.0) {
        RCLCPP_WARN(this->get_logger(), "[GAS] Bubbles detected: %.2f. Activating valve.", gas_bubbles_);
        open_three_way_valve();
    }
}

void IonizationBed::open_three_way_valve() {
    RCLCPP_INFO(this->get_logger(), "[VALVE] Three-way valve engaged. Diverting bubbles.");
    gas_bubbles_ = 0.0;
}

void IonizationBed::send_to_electrolysis() {
    if (!electrolysis_client_->wait_for_service(5s)) {
        RCLCPP_FATAL(this->get_logger(), "[FATAL] Electrolysis service unavailable!");
        return;
    }

    auto request = std::make_shared<space_station_eclss::srv::Water::Request>();
    request->water = water_;
    request->contaminants = contaminants_;
    request->iodine_level = iodine_level_;
    request->gas_bubbles = gas_bubbles_;
    request->pressure = 14.0;
    request->temperature = 25.0;

    RCLCPP_INFO(this->get_logger(), "[SEND] To Electrolysis: %.2f L | Contaminants = %.2f | Iodine = %.2f | Gas = %.2f", 
                water_, contaminants_, iodine_level_, gas_bubbles_);

    auto future = electrolysis_client_->async_send_request(request,
        [this](rclcpp::Client<space_station_eclss::srv::Water>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "[SUCCESS] Electrolysis accepted water.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "[FAIL] Electrolysis rejected water: %s", response->message.c_str());
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "[EXCEPTION] Electrolysis service failed: %s", e.what());
            }

            is_active = false;
            RCLCPP_INFO(this->get_logger(), "[STATE] Ionization bed ready for next request.");
        });

    water_ = 0.0;
    contaminants_ = 0.0;
    iodine_level_ = 0.0;
    gas_bubbles_ = 0.0;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IonizationBed>());
    rclcpp::shutdown();
    return 0;
}
