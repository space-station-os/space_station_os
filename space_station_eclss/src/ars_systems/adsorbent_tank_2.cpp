#include "space_station_eclss/ars_system/adsorbent_tank.hpp"
#include <cmath>

using namespace std::chrono_literals;

AdsorbentBed2::AdsorbentBed2()
: Node("adsorbent_bed2"), is_active_(false), co2_buffer_(0.0),has_heated_(false)
{
  this->declare_parameter("desorption_temperature", 400.0);

  bed2_service_ = this->create_service<space_station_eclss::srv::CrewQuarters>(
    "/adsorbent_bed2",
    std::bind(&AdsorbentBed2::handle_request, this, std::placeholders::_1, std::placeholders::_2));

  co2_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/co2_storage", 10);
  cdra_status_publisher_ = this->create_publisher<space_station_eclss::msg::CdraStatus>("/cdra_status", 10);
  cdra = space_station_eclss::msg::CdraStatus();

  RCLCPP_INFO(this->get_logger(), "Adsorbent Bed 2 ready for desorption.");
}

void AdsorbentBed2::handle_request(
  const std::shared_ptr<space_station_eclss::srv::CrewQuarters::Request> request,
  std::shared_ptr<space_station_eclss::srv::CrewQuarters::Response> response)
{
  if (is_active_) {
    response->success = false;
    response->message = "Bed 2 is already active.";
    return;
  }

  if (request->co2_mass <= 0.01) {
    RCLCPP_WARN(this->get_logger(), "[Bed 2] Received request with no CO₂. Ignoring.");
    response->success = false;
    response->message = "No CO₂ received.";
    return;
  }

  co2_buffer_ = request->co2_mass;
  temperature_ = 300.0; // reset temp
  has_heated_ = false;
  is_active_ = true;

  cdra.co2_processing_state = 6;
  cdra.system_health = 1;
  cdra.data = "Starting Desorption";
  cdra_status_publisher_->publish(cdra);

  RCLCPP_INFO(this->get_logger(), "[Bed 2] Received %.2f g CO₂ for desorption", co2_buffer_);
  RCLCPP_INFO(this->get_logger(), "[Bed 2] Heating to desorption temperature...");

  timer_ = this->create_wall_timer(1s, std::bind(&AdsorbentBed2::desorb_co2, this));

  response->success = true;
  response->message = "Desorption started.";
}

void AdsorbentBed2::desorb_co2() {
  std::lock_guard<std::mutex> lock(data_mutex_);

  // 1. Dynamic desorption rate based on mode
  std::string mode;
  this->get_parameter_or("mode_of_operation", mode, std::string("idle"));

  std::map<std::string, double> desorption_rates = {
    {"idle", 0.03},
    {"standby", 0.04},
    {"exercise", 0.06},
    {"emergency", 0.08},
    {"biological_research", 0.05},
    {"eva_repair", 0.07}
  };
  double k_des = desorption_rates.count(mode) ? desorption_rates[mode] : 0.05;

  double desorption_temp;
  this->get_parameter("desorption_temperature", desorption_temp);

  // 2. Heat once and hold
  if (!has_heated_ && temperature_ < desorption_temp) {
    temperature_ = desorption_temp;
    has_heated_ = true;
    RCLCPP_INFO(this->get_logger(), "[Bed 2] Temperature set to %.2f°C for desorption.", temperature_);
    return;
  }

  // 3. Safety check
  if (co2_buffer_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "[Bed 2] No CO₂ to store. Skipping desorption.");
    cdra.co2_processing_state = 7;
    cdra.system_health = 2;
    cdra.data = "Desorption failed: No CO₂ available";
    cdra_status_publisher_->publish(cdra);
    is_active_ = false;
    timer_->cancel();
    return;
  }

  // 4. Langmuir-style desorption
  double desorbed = k_des * co2_buffer_;
  desorbed = std::min(desorbed, co2_buffer_);
  co2_buffer_ -= desorbed;

  static double total_co2_stored = 0.0;
  total_co2_stored += desorbed;

  std_msgs::msg::Float64 stored_msg;
  stored_msg.data = total_co2_stored;
  co2_publisher_->publish(stored_msg);

  RCLCPP_INFO(this->get_logger(),
              "[Bed 2] Mode: %s | k_des: %.3f | Desorbed %.2f g | CO₂ remaining: %.2f g | Stored total: %.2f g",
              mode.c_str(), k_des, desorbed, co2_buffer_, total_co2_stored);

  if (co2_buffer_ <= 4.0) {
    RCLCPP_INFO(this->get_logger(), "[Bed 2] Desorption complete. All CO₂ stored.");
    cdra.co2_processing_state = 7;
    cdra.system_health = 1;
    cdra.data = "CO₂ fully transferred to storage";
    cdra_status_publisher_->publish(cdra);

    co2_buffer_ = 0.0;
    is_active_ = false;
    temperature_ = 300.0;
    has_heated_ = false;
    timer_->cancel();
  }
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdsorbentBed2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
