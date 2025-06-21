#include "space_station_eclss/waste_tank.h"
#include "std_msgs/msg/float64.hpp"

WHCWasteTank::WHCWasteTank() 
    : Node("whc_waste_tank"),
      urine_volume_(0.3), 
      pretreatment_volume_(0.05), 
      flush_volume_(1.0),
      total_water_volume_(0.0),
      upa_available_(true) {

    this->declare_parameter("tank_capacity", 22.0);
    this->declare_parameter("processing_threshold", 10.0);

    tank_capacity_ = this->get_parameter("tank_capacity").as_double();
    processing_threshold_ = this->get_parameter("processing_threshold").as_double();

    upa_client_ = this->create_client<space_station_eclss::srv::Upa>("/upa/process_urine");

    retry_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&WHCWasteTank::retry_process_waste_transfer, this));

    urine_collection_timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&WHCWasteTank::simulate_urine_collection, this));
    
    dashboard_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&WHCWasteTank::publish_status, this));

    waste_status_pub_ = this->create_publisher<space_station_eclss::msg::WaterCrew>("/whc/collector_status", 10);

    grey_water_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/grey_water", 10,
        std::bind(&WHCWasteTank::receive_grey_water, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[INIT] WHC Waste Tank Node Initialized");
}

void WHCWasteTank::publish_status() {
    auto msg = space_station_eclss::msg::WaterCrew();
    msg.water = total_water_volume_;
    msg.gas_bubbles = 0.0;         
    msg.contaminants = 0.8 * total_water_volume_;  
    msg.iodine_level = 0.02;        
    msg.pressure = 101.3;           
    msg.temperature = 22.5;        
    waste_status_pub_->publish(msg);
}

void WHCWasteTank::simulate_urine_collection() {
    if (!upa_available_) {
        RCLCPP_WARN(this->get_logger(), "[BLOCKED] UPA unavailable. Holding urine.");
        return;
    }

    double new_waste = urine_volume_ + pretreatment_volume_ + flush_volume_;
    total_water_volume_ += new_waste;

    if (total_water_volume_ >= processing_threshold_) {
        RCLCPP_INFO(this->get_logger(), "[THRESHOLD] Total waste reached %.2f L. Sending to UPA.", total_water_volume_);
        process_waste_transfer();
    }
}

void WHCWasteTank::receive_grey_water(const std_msgs::msg::Float64::SharedPtr msg) {
    total_water_volume_ += msg->data;
    RCLCPP_INFO(this->get_logger(), "[INPUT] +%.2f L from Grey Water Tank → Total: %.2f L", msg->data, total_water_volume_);

    if (total_water_volume_ >= processing_threshold_) {
        RCLCPP_INFO(this->get_logger(), "[THRESHOLD] Total waste reached %.2f L. Sending to UPA.", total_water_volume_);
        process_waste_transfer();
    }
}

void WHCWasteTank::process_waste_transfer() {
    if (!upa_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_FATAL(this->get_logger(), "[FAIL] UPA service not reachable. Will retry...");
        upa_available_ = false;
        return;
    }

    if (!upa_available_) {
        RCLCPP_INFO(this->get_logger(), "[RECOVERY] UPA service back online. Resuming urine transfer.");
        upa_available_ = true;
    }

    auto request = std::make_shared<space_station_eclss::srv::Upa::Request>();
    request->urine = total_water_volume_;

    RCLCPP_INFO(this->get_logger(), "[SEND] Dispatching %.2f L of waste to UPA...", total_water_volume_);

    auto future_result = upa_client_->async_send_request(
        request, std::bind(&WHCWasteTank::process_urine_response, this, std::placeholders::_1));
}

void WHCWasteTank::process_urine_response(rclcpp::Client<space_station_eclss::srv::Upa>::SharedFuture future) {
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "[UPA] Success: %s", response->message.c_str());
            total_water_volume_ = 0.0;
        } else {
            RCLCPP_WARN(this->get_logger(), "[UPA] Failed: %s. Will retry...", response->message.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Exception during UPA call: %s", e.what());
    }
}

void WHCWasteTank::retry_process_waste_transfer() {
    if (total_water_volume_ >= processing_threshold_) {
        RCLCPP_WARN(this->get_logger(), "[RETRY] Attempting waste resend to UPA...");
        process_waste_transfer();
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WHCWasteTank>());
    rclcpp::shutdown();
    return 0;
}
