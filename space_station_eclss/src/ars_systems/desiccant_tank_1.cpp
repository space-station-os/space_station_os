#include "space_station_eclss/ars_system/desiccant_tank.hpp"
#include <mutex>
#include <algorithm>
#include <unordered_map>

using namespace std::chrono_literals;

DesiccantBed1::DesiccantBed1()
: Node("desiccant_bed1"), is_active_(false), retry_count_(0), max_retries_(10)
{
  this->declare_parameter("mode_of_operation", "idle");
  this->declare_parameter("moisture_removal_rate", 0.95);
  this->declare_parameter("contaminant_removal_rate", 0.90);

  bed1_service_ = this->create_service<space_station_eclss::srv::CrewQuarters>(
    "/desiccant_bed1",
    std::bind(&DesiccantBed1::handle_request, this, std::placeholders::_1, std::placeholders::_2));

  adsorbent_client_ = this->create_client<space_station_eclss::srv::CrewQuarters>("/adsorbent_bed1");
  air_quality_publisher_ = this->create_publisher<space_station_eclss::msg::AirData>("/desiccant_air_quality", 10);

  RCLCPP_INFO(this->get_logger(), "Desiccant Bed 1 Initialized.");
}

void DesiccantBed1::handle_request(
  const std::shared_ptr<space_station_eclss::srv::CrewQuarters::Request> request,
  std::shared_ptr<space_station_eclss::srv::CrewQuarters::Response> response)
{
  if (is_active_) {
    response->success = false;
    response->message = "Bed 1 busy.";
    return;
  }

  co2_ = request->co2_mass;
  moisture_ = request->moisture_content;
  contaminants_ = request->contaminants;
  retry_count_ = 0;
  is_active_ = true;

  RCLCPP_INFO(this->get_logger(), "[Bed 1] Received: CO₂=%.2f, Moisture=%.2f%%, Contaminants=%.2f%%",
              co2_, moisture_, contaminants_);
  
  auto msg = space_station_eclss::msg::AirData();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "desiccant_bed_1";
  msg.co2_mass = co2_;
  msg.moisture_content = moisture_;
  msg.contaminants = contaminants_;
  msg.temperature = 0.0;
  msg.dew_point = 0.0;
  air_quality_publisher_->publish(msg);

  timer_ = this->create_wall_timer(1s, std::bind(&DesiccantBed1::process_air, this));
  response->success = true;
  response->message = "Bed 1 processing started.";
}

void DesiccantBed1::process_air()
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Self-adjust logic based on mode
  std::string mode;
  this->get_parameter("mode_of_operation", mode);

  const std::unordered_map<std::string, double> moisture_rates = {
    {"idle", 0.20}, {"standby", 0.30}, {"exercise", 0.50},
    {"emergency", 0.65}, {"biological_research", 0.35}, {"eva_repair", 0.40}
  };

  const std::unordered_map<std::string, double> contaminant_rates = {
    {"idle", 0.15}, {"standby", 0.25}, {"exercise", 0.40},
    {"emergency", 0.55}, {"biological_research", 0.30}, {"eva_repair", 0.45}
  };

  double rate_m = moisture_rates.count(mode) ? moisture_rates.at(mode) : 0.25;
  double rate_c = contaminant_rates.count(mode) ? contaminant_rates.at(mode) : 0.20;

  moisture_ *= (1.0 - rate_m);
  contaminants_ *= (1.0 - rate_c);

  moisture_ = std::max(0.0, moisture_);
  contaminants_ = std::max(0.0, contaminants_);
  auto msg = space_station_eclss::msg::AirData();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "desiccant_bed_1";
  msg.co2_mass = co2_;
  msg.moisture_content = moisture_;
  msg.contaminants = contaminants_;
  msg.temperature = 0.0;
  msg.dew_point = 0.0;
  air_quality_publisher_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "[Bed 1] Mode: %s | Moisture: %.2f%%, Contaminants: %.2f%%", mode.c_str(), moisture_, contaminants_);

  if (moisture_ <= 15.0 && contaminants_ <= 2.0) {
    if (!adsorbent_client_->wait_for_service(2s)) {
      RCLCPP_WARN(this->get_logger(), "[Bed 1] Adsorbent bed service unavailable. Will retry...");
      retry_count_++;
      if (retry_count_ >= max_retries_) {
        RCLCPP_ERROR(this->get_logger(), "[Bed 1] Max retries reached. Aborting.");
        is_active_ = false;
        timer_->cancel();
      }
      return;
    }

    auto request = std::make_shared<space_station_eclss::srv::CrewQuarters::Request>();
    request->co2_mass = co2_;
    request->moisture_content = moisture_;
    request->contaminants = contaminants_;

    adsorbent_client_->async_send_request(request,
      [this](rclcpp::Client<space_station_eclss::srv::CrewQuarters>::SharedFuture future) {
        auto res = future.get();
        if (res->success) {
          RCLCPP_INFO(this->get_logger(), "[Bed 1] Sent air to Adsorbent Bed.");
          is_active_ = false;
          retry_count_ = 0;
          timer_->cancel();
        } else {
          retry_count_++;
          RCLCPP_WARN(this->get_logger(), "[Bed 1] Failed to send air. Will retry... (%d/%d)", retry_count_, max_retries_);
          if (retry_count_ >= max_retries_) {
            RCLCPP_ERROR(this->get_logger(), "[Bed 1] Max retries reached. Aborting.");
            is_active_ = false;
            timer_->cancel();
          }
        }
      });
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DesiccantBed1>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
