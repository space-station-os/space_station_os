#include "space_station_eclss/ars_system/adsorbent_tank.hpp"
#include <cmath>
#include <mutex>

using namespace std::chrono_literals;

AdsorbentBed1::AdsorbentBed1()
: Node("adsorbent_bed_1"), is_active_(false), co2_buffer_(0.0), previous_error_(0.0), temperature_(300.0)
{
  this->declare_parameter("co2_adsorption_rate_constant", 0.01);
  this->declare_parameter("co2_capacity", 150.0);
  this->declare_parameter("desired_temperature", 420.0);
  this->declare_parameter("temperature_tolerance", 30.0);
  this->declare_parameter("kp", 0.6);
  this->declare_parameter("kd", 0.15);
  this->declare_parameter("tank_capacity", 500.0);
  this->declare_parameter("mode_of_operation", "standby");

  bed1_service_ = this->create_service<space_station_eclss::srv::CrewQuarters>(
    "/adsorbent_bed1",
    std::bind(&AdsorbentBed1::handle_request, this, std::placeholders::_1, std::placeholders::_2));
  desorption_client_ = this->create_client<space_station_eclss::srv::CrewQuarters>("/adsorbent_bed2");
  desiccant_client_ = this->create_client<space_station_eclss::srv::CrewQuarters>("/desiccant_bed2");

  cdra_status_publisher_ = this->create_publisher<space_station_eclss::msg::CdraStatus>("/cdra_status", 10);
  air_quality_publisher_ = this->create_publisher<space_station_eclss::msg::AirData>("/adsorbent_air_quality", 10);

  cdra = space_station_eclss::msg::CdraStatus();

  RCLCPP_INFO(this->get_logger(), "Adsorbent Bed 1 with Langmuir kinetics initialized.");
}

void AdsorbentBed1::handle_request(
  const std::shared_ptr<space_station_eclss::srv::CrewQuarters::Request> request,
  std::shared_ptr<space_station_eclss::srv::CrewQuarters::Response> response)
{
  if (is_active_) {
    response->success = false;
    response->message = "Adsorbent Bed 1 busy.";
    return;
  }

  is_active_ = true;
  co2_ = request->co2_mass;
  moisture_ = request->moisture_content;
  contaminants_ = request->contaminants;
  co2_buffer_ = 0.0; // Reset buffer for fresh batch

  RCLCPP_INFO(this->get_logger(), "Received air for adsorption: CO₂=%.2f g, Moisture=%.2f%%, Contaminants=%.2f%%",
              co2_, moisture_, contaminants_);

  timer_ = this->create_wall_timer(1s, std::bind(&AdsorbentBed1::process_adsorption,this));
  response->success = true;
  response->message = "Adsorption started.";
}

void AdsorbentBed1::process_adsorption()
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  std::string mode;
  this->get_parameter("mode_of_operation", mode);

  std::map<std::string, double> k_ads_map = {
    {"idle", 0.008},
    {"standby", 0.010},
    {"exercise", 0.020},
    {"emergency", 0.025},
    {"biological_research", 0.015},
    {"eva_repair", 0.022}
  };

  double k_ads = k_ads_map.count(mode) ? k_ads_map.at(mode) : 0.010;
  double C_max, T_target, T_tol, kp, kd;
  this->get_parameter("co2_capacity", C_max);
  this->get_parameter("desired_temperature", T_target);
  this->get_parameter("temperature_tolerance", T_tol);
  this->get_parameter("kp", kp);
  this->get_parameter("kd", kd);
  this->get_parameter("tank_capacity", tank_capacity_);

  double error = T_target - temperature_;
  double derivative = error - previous_error_;
  double control = kp * error + kd * derivative;
  temperature_ += control;
  previous_error_ = error;

  if (temperature_ < (T_target - T_tol)) {
    RCLCPP_WARN(this->get_logger(), "[Bed 1] Temperature %.2f°C too low for adsorption.", temperature_);
    return;
  }

  double delta_q = (C_max - co2_buffer_) * (k_ads * co2_ / (1.0 + k_ads * co2_));
  delta_q = std::min(delta_q, co2_);
  co2_ -= delta_q;
  co2_buffer_ += delta_q;

  RCLCPP_INFO(this->get_logger(), "[Bed 1] Mode: %s | k_ads: %.3f | Adsorbed %.5f g | Remaining: %.2f g | Buffer: %.2f g",
              mode.c_str(), k_ads, delta_q, co2_, co2_buffer_);

  auto msg = space_station_eclss::msg::AirData();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "adsorbent_bed_1";
  msg.co2_mass = co2_buffer_;
  msg.moisture_content = moisture_;
  msg.contaminants = contaminants_;
  air_quality_publisher_->publish(msg);

  if (co2_ <= 4.0) {
    RCLCPP_INFO(this->get_logger(), "[Bed 1] Adsorption complete. CO₂ fully captured.");
    is_active_ = false;
    timer_->cancel();

    if (desorption_client_->wait_for_service(2s)) {
      auto req = std::make_shared<space_station_eclss::srv::CrewQuarters::Request>();
      req->co2_mass = co2_buffer_;
      req->moisture_content = moisture_;
      req->contaminants = contaminants_;
      desorption_client_->async_send_request(req);
      RCLCPP_INFO(this->get_logger(), "[Bed 1] Triggered desorption.");
    } else {
      RCLCPP_WARN(this->get_logger(), "[Bed 1] Desorption service unavailable.");
    }

    auto request = std::make_shared<space_station_eclss::srv::CrewQuarters::Request>();
    request->co2_mass = co2_;
    request->moisture_content = moisture_;
    request->contaminants = contaminants_;

    if (!desiccant_client_->wait_for_service(2s)) {
      RCLCPP_WARN(this->get_logger(), "[Bed 1] Desiccant Bed 2 unavailable for humidification.");
    } else {
      desiccant_client_->async_send_request(request,
        [this](rclcpp::Client<space_station_eclss::srv::CrewQuarters>::SharedFuture future) {
          if (future.get()->success) {
            RCLCPP_INFO(this->get_logger(), "[Bed 1] Sent air to Desiccant Bed 2 for humidification.");
          } else {
            RCLCPP_WARN(this->get_logger(), "[Bed 1] Desiccant Bed 2 failed to accept air.");
          }
        });
    }
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdsorbentBed1>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
