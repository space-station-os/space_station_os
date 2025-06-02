#include "space_station_eclss/ors_system/water_req.h"

using namespace std::chrono_literals;

WaterService::WaterService()
    : Node("water_service"),
      water_level_(0.0),
      contaminants_level_(0.0),
      iodine_level_(0.0),
      gas_bubbles_(0.0),
      pressure_(14.7),
      temperature_(25.0),
      ready_for_next_request_(true),
      max_tank_capacity_(this->declare_parameter<double>("max_tank_capacity", 2000.0)),
      accumulation_rate_(this->declare_parameter<double>("accumulation_rate", 100.0)),
      threshold_level_(this->declare_parameter<double>("threshold_level", 1000.0)) {

    tank_status_subscriber_ = this->create_subscription<space_station_eclss::msg::WaterCrew>(
        "/wpa/tank_status", 10,
        std::bind(&WaterService::tank_status_callback, this, std::placeholders::_1)
    );

    water_service_ = this->create_service<space_station_eclss::srv::Water>(
        "/water_request",
        std::bind(&WaterService::handle_water_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    deionization_client_ = this->create_client<space_station_eclss::srv::Water>("/deionization_chamber");

    accumulation_timer_ = this->create_wall_timer(
        2s, std::bind(&WaterService::water_accumulation, this)
    );

    RCLCPP_INFO(this->get_logger(), "[INIT] Water Service Node Initialized. Awaiting water from Product Tank...");
}

void WaterService::tank_status_callback(const space_station_eclss::msg::WaterCrew::SharedPtr msg) {
    double extracted = msg->water * 0.2;
    if (water_level_ + extracted > max_tank_capacity_) {
        RCLCPP_WARN(this->get_logger(), "[SKIP] Tank at capacity (%.2f L).", water_level_);
        return;
    }

    water_level_ += extracted;
    contaminants_level_ += msg->contaminants * 0.2;
    iodine_level_ += msg->iodine_level * 0.2;
    gas_bubbles_ += msg->gas_bubbles * 0.2;

    RCLCPP_INFO(this->get_logger(), "[INTAKE] +%.2f L from Product Tank → Total = %.2f L", extracted, water_level_);

    if (water_level_ >= threshold_level_ && ready_for_next_request_) {
        RCLCPP_INFO(this->get_logger(), "[THRESHOLD] %.2f L reached. Triggering deionization...", water_level_);
        send_deionization_request();
        ready_for_next_request_ = false;
    }
}

void WaterService::water_accumulation() {
    if (water_level_ >= max_tank_capacity_) {
        RCLCPP_WARN(this->get_logger(), "[BLOCK] Simulated tank full (%.2f L).", water_level_);
        return;
    }

    water_level_ += accumulation_rate_;
    contaminants_level_ += accumulation_rate_ * 0.2;
    iodine_level_ += accumulation_rate_ * 0.05;

    if (water_level_ > max_tank_capacity_)
        water_level_ = max_tank_capacity_;

    RCLCPP_INFO(this->get_logger(), "[FILL] Simulated fill → Water = %.2f L, Contaminants = %.2f, Iodine = %.2f",
                water_level_, contaminants_level_, iodine_level_);

    if (water_level_ >= threshold_level_ && ready_for_next_request_) {
        RCLCPP_INFO(this->get_logger(), "[THRESHOLD] %.2f L reached. Triggering deionization...", water_level_);
        send_deionization_request();
        ready_for_next_request_ = false;
    }
}

void WaterService::handle_water_request(
    const std::shared_ptr<space_station_eclss::srv::Water::Request> request,
    std::shared_ptr<space_station_eclss::srv::Water::Response> response) {

    RCLCPP_INFO(this->get_logger(), "[REQ] Received external water request: %.2f L", request->water);

    if (water_level_ < request->water) {
        response->success = false;
        response->message = "Not enough water available.";
        RCLCPP_ERROR(this->get_logger(), "[FAIL] Insufficient water.");
        return;
    }

    send_deionization_request();
    response->success = true;
    response->message = "Water transferred to Deionization Chamber.";
    ready_for_next_request_ = false;
}

void WaterService::send_deionization_request() {
    if (!deionization_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Deionization service unavailable!");
        return;
    }

    auto request = std::make_shared<space_station_eclss::srv::Water::Request>();
    request->water = water_level_;
    request->contaminants = contaminants_level_;
    request->iodine_level = iodine_level_;
    request->gas_bubbles = gas_bubbles_;
    request->pressure = pressure_;
    request->temperature = temperature_;

    RCLCPP_INFO(this->get_logger(), "[SEND] Dispatching %.2f L to Deionization Chamber", water_level_);

    auto future = deionization_client_->async_send_request(
        request, std::bind(&WaterService::process_deionization_response, this, std::placeholders::_1));
}

void WaterService::process_deionization_response(rclcpp::Client<space_station_eclss::srv::Water>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "[SUCCESS] Water transferred. Tank emptied.");
            RCLCPP_DEBUG(this->get_logger(), "[DEBUG] Resetting internal tank values to 0.");
            water_level_ = 0.0;
            contaminants_level_ = 0.0;
            iodine_level_ = 0.0;
            gas_bubbles_ = 0.0;
            ready_for_next_request_ = true;

            accumulation_timer_->cancel();
            rclcpp::sleep_for(5s);
            accumulation_timer_ = this->create_wall_timer(
                2s, std::bind(&WaterService::water_accumulation, this));
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "[FAIL] Deionization error: %s", response->message.c_str());
            ready_for_next_request_ = true;
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "[EXCEPTION] While sending to deionization: %s", e.what());
        ready_for_next_request_ = true;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaterService>());
    rclcpp::shutdown();
    return 0;
}
