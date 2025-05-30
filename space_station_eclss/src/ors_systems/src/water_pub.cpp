#include "space_station_eclss/water_status.h"

using namespace std::chrono_literals;

WaterService::WaterService()
    : Node("water_service"),
      water_level_(0.0),
      contaminants_level_(0.0),
      iodine_level_(0.0),
      gas_bubbles_(0.0),
      pressure_(14.7),
      temperature_(25.0),
      max_tank_capacity_(this->declare_parameter<double>("max_tank_capacity", 20.0)),  // Reduced tank capacity
      accumulation_rate_(this->declare_parameter<double>("accumulation_rate", 10.0)),
      threshold_level_(this->declare_parameter<double>("threshold_level", 15.0)) {

    // Subscribe to tank status updates
    tank_status_subscriber_ = this->create_subscription<space_station_eclss::msg::WaterCrew>(
        "/wpa/tank_status", 10, std::bind(&WaterService::tank_status_callback, this, std::placeholders::_1)
    );

    // Service to provide water when requested
    water_service_ = this->create_service<space_station_eclss::srv::Water>(
        "/water_request",
        std::bind(&WaterService::handle_water_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Client to send water to deionization chamber
    deionization_client_ = this->create_client<space_station_eclss::srv::Water>("/deionization_chamber");

    // Timer to simulate water accumulation over time
    accumulation_timer_ = this->create_wall_timer(
        2s, std::bind(&WaterService::water_accumulation, this)
    );

    RCLCPP_INFO(this->get_logger(), "Water service node initialized. Monitoring tank levels...");
}

void WaterService::tank_status_callback(const space_station_eclss::msg::WaterCrew::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received tank status update.");

    // Extract only 20% of the total water contents from the message
    double extracted_water = msg->water * 0.2;
    double extracted_contaminants = msg->contaminants * 0.2;
    double extracted_iodine = msg->iodine_level * 0.2;
    double extracted_gas_bubbles = msg->gas_bubbles * 0.2;

    if (water_level_ + extracted_water > max_tank_capacity_) {
        RCLCPP_WARN(this->get_logger(), "Cannot extract more water. Tank is nearing capacity.");
        return;
    }

    // Update water storage levels
    water_level_ += extracted_water;
    contaminants_level_ += extracted_contaminants;
    iodine_level_ += extracted_iodine;
    gas_bubbles_ += extracted_gas_bubbles;

    RCLCPP_INFO(this->get_logger(), "Extracted %.2fL from the main tank. Current water level: %.2fL",
                extracted_water, water_level_);
}

void WaterService::water_accumulation() {
    if (water_level_ >= max_tank_capacity_) {
        RCLCPP_WARN(this->get_logger(), "Tank full (%.2f L)! Sending request to deionization chamber.", water_level_);
        send_deionization_request();
        return;
    }

    // Increment water level, contaminants, and iodine concentration
    water_level_ += accumulation_rate_;
    contaminants_level_ += accumulation_rate_ * 0.2;
    iodine_level_ += accumulation_rate_ * 0.05;

    if (water_level_ > max_tank_capacity_) {
        water_level_ = max_tank_capacity_;
    }
    RCLCPP_INFO(this->get_logger(), "===========================");
    RCLCPP_INFO(this->get_logger(), "Tank filling: Water = %.2f L, Contaminants = %.2f ppm, Iodine = %.2f ppm",
                water_level_, contaminants_level_, iodine_level_);
    RCLCPP_INFO(this->get_logger(), "===========================");
}

void WaterService::handle_water_request(
    const std::shared_ptr<space_station_eclss::srv::Water::Request> request,
    std::shared_ptr<space_station_eclss::srv::Water::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Received external water request for %.2f L", request->water);

    if (water_level_ < request->water) {
        response->success = false;
        response->message = "Not enough water available!";
        RCLCPP_ERROR(this->get_logger(), "Request failed: Insufficient water.");
        return;
    }

    send_deionization_request();
    response->success = true;
    response->message = "Water successfully transferred to deionization chamber.";
}

void WaterService::send_deionization_request() {
    if (!deionization_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(this->get_logger(), "Deionization chamber service is not available!");
        return;
    }

    auto request = std::make_shared<space_station_eclss::srv::Water::Request>();
    request->water = water_level_;
    request->contaminants = contaminants_level_;
    request->iodine_level = iodine_level_;
    request->gas_bubbles = gas_bubbles_;
    request->pressure = 14.7;
    request->temperature = 25.0;

    RCLCPP_INFO(this->get_logger(), "Sending water to deionization chamber: Water = %.2f L, Contaminants = %.2f ppm, Iodine = %.2f ppm",
                request->water, request->contaminants, request->iodine_level);

    auto future = deionization_client_->async_send_request(request,
        std::bind(&WaterService::process_deionization_response, this, std::placeholders::_1));
}

void WaterService::process_deionization_response(rclcpp::Client<space_station_eclss::srv::Water>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Deionization successful. Emptying tank...");
            water_level_ = 0.0;
            contaminants_level_ = 0.0;
            iodine_level_ = 0.0;
            gas_bubbles_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Tank emptied successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Deionization failed: %s", response->message.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while calling deionization service: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaterService>());
    rclcpp::shutdown();
    return 0;
}
