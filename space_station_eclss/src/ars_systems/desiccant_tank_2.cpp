#include "space_station_eclss/ars_system/desiccant_tank.hpp"
#include <mutex>

using namespace std::chrono_literals;

DesiccantBed2::DesiccantBed2()
: Node("desiccant_bed2"), is_active_(false), moisture_(0.0)
{
  this->declare_parameter("humidification_rate", 1.5);

  bed2_service_ = this->create_service<space_station_eclss::srv::CrewQuarters>(
    "/desiccant_bed2",
    std::bind(&DesiccantBed2::handle_request, this, std::placeholders::_1, std::placeholders::_2));

  air_quality_publisher_ = this->create_publisher<space_station_eclss::msg::AirData>("/desiccant_air_quality", 10);

  RCLCPP_INFO(this->get_logger(), "Desiccant Bed 2 Initialized.");
}

void DesiccantBed2::handle_request(
  const std::shared_ptr<space_station_eclss::srv::CrewQuarters::Request> request,
  std::shared_ptr<space_station_eclss::srv::CrewQuarters::Response> response)
{
  if (is_active_) {
    response->success = false;
    response->message = "Bed 2 is busy.";
    return;
  }

  co2_ = request->co2_mass;
  moisture_ = 0.0;
  contaminants_ = request->contaminants;
  is_active_ = true;

  RCLCPP_INFO(this->get_logger(), "[Bed 2] Starting humidification...");

  timer_ = this->create_wall_timer(1s, std::bind(&DesiccantBed2::humidify_air, this));

  response->success = true;
  response->message = "Humidification started.";
}

void DesiccantBed2::humidify_air()
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Dynamic humidification rate based on mode
  std::string mode;
  this->get_parameter_or("mode_of_operation", mode, std::string("idle"));

  std::map<std::string, double> humidification_rates = {
    {"idle", 0.8},
    {"standby", 1.0},
    {"exercise", 1.8},
    {"emergency", 2.5},
    {"biological_research", 1.2},
    {"eva_repair", 2.0}
  };

  double rate = humidification_rates.count(mode) ? humidification_rates[mode] : 1.5;

  moisture_ += rate;
  if (moisture_ > 80.0) moisture_ = 80.0;

  RCLCPP_INFO(this->get_logger(), "[Bed 2] Mode: %s | Rate: %.2f | Moisture level: %.2f%%",
              mode.c_str(), rate, moisture_);

  if (moisture_ >= 80.0) {
    auto msg = space_station_eclss::msg::AirData();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "desiccant_bed_2";
    msg.co2_mass = co2_;
    msg.moisture_content = moisture_;
    msg.contaminants = contaminants_;
    msg.temperature = 0.0;
    msg.dew_point = 0.0;
    air_quality_publisher_->publish(msg);

    is_active_ = false;
    timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "[Bed 2] Humidification complete and published air.");
  }
}



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DesiccantBed2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  }
  