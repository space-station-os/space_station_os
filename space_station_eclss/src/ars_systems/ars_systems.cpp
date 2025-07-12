#include "space_station_eclss/ars_systems.hpp"

using namespace std::chrono_literals;
using space_station_eclss::ARSActionServer;

ARSActionServer::ARSActionServer(const rclcpp::NodeOptions & options)
: Node("air_revitalisation", options)
{
  std::srand(std::time(nullptr)); 
  declare_parameters();

  action_server_ = rclcpp_action::create_server<AirRevitalisation>(
    this,
    "air_revitalisation",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const AirRevitalisation::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<GoalHandleARS>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    std::bind(&ARSActionServer::execute, this, std::placeholders::_1)
  );

  co2_storage_timer_ = this->create_wall_timer(
      1s,  // Publish every 1s
      [this]() {
        std_msgs::msg::Float64 msg;
        msg.data = this->total_co2_storage_;
        co2_storage_pub_->publish(msg);
      }
    );

  heartbeat_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/ars/heartbeat", 10);
  co2_storage_pub_ = this->create_publisher<std_msgs::msg::Float64>("/co2_storage", 10);


  RCLCPP_INFO(this->get_logger(), "ARS Action Server Initialized");
}

void ARSActionServer::declare_parameters()
{
  sim_time_ = this->declare_parameter<int>("sim_time", 1000);
  enable_failure_ = this->declare_parameter("enable_failure", false);
  max_co2_storage_ = this->declare_parameter("max_co2_storage", 3000.0);

  des1_capacity_ = this->declare_parameter("desiccant_bed_1.capacity", 50.0);
  des1_removal_  = this->declare_parameter("desiccant_bed_1.removal_rate", 0.1);
  des1_temp_limit_ = this->declare_parameter("desiccant_bed_1.max_temperature", 80.0);

  des2_capacity_ = this->declare_parameter("desiccant_bed_2.capacity", 50.0);
  des2_removal_  = this->declare_parameter("desiccant_bed_2.removal_rate", 0.1);
  des2_temp_limit_ = this->declare_parameter("desiccant_bed_2.max_temperature", 80.0);

  ads1_capacity_ = this->declare_parameter("adsorbent_bed_1.capacity", 500.0);
  ads1_removal_  = this->declare_parameter("adsorbent_bed_1.removal_rate", 5.0);
  ads1_temp_limit_ = this->declare_parameter("adsorbent_bed_1.max_temperature", 204.0);

  ads2_capacity_ = this->declare_parameter("adsorbent_bed_2.capacity", 500.0);
  ads2_removal_  = this->declare_parameter("adsorbent_bed_2.removal_rate", 5.0);
  ads2_temp_limit_ = this->declare_parameter("adsorbent_bed_2.max_temperature", 204.0);
}

void ARSActionServer::execute(const std::shared_ptr<GoalHandleARS> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<AirRevitalisation::Feedback>();
  auto result = std::make_shared<AirRevitalisation::Result>();

  float co2 = goal->initial_co2_mass;
  float h2o = goal->initial_moisture_content;

  float t_ads1 = 140.0, t_ads2 = 140.0;
  float t_des1 = 30.0, t_des2 = 30.0;

  int cycles = 0;
  int vents = 0;
  float total_vented = 0.0;
  bool success = true;

  RCLCPP_INFO(this->get_logger(), "[Goal] ARS simulation started");

  for (int t = 0; t <= sim_time_; t++) {
    // [1] Check CO2 storage overflow failure condition
    if (enable_failure_ && total_co2_storage_ > max_co2_storage_) {
      RCLCPP_FATAL(this->get_logger(), "[FAILURE] CO₂ leak into crew quarters! Storage exceeded %.2f ppm", total_co2_storage_);
      publish_bed_heartbeat("CO₂ Storage", false, "CO₂ leak detected due to overcapacity");
      success = false;
      break;
    }

    feedback->current_time = t;
    feedback->cabin_co2_level = co2;

    if (!simulate_desiccant_bed(h2o, des1_capacity_, des1_removal_, des1_temp_limit_, "Desiccant Bed 1", t_des1)) {
      publish_bed_heartbeat("Desiccant Bed 1", false, "H₂O capacity depleted");
      success = false;
      break;
    }

    if (!simulate_desiccant_bed(h2o, des2_capacity_, des2_removal_, des2_temp_limit_, "Desiccant Bed 2", t_des2)) {
      publish_bed_heartbeat("Desiccant Bed 2", false, "H₂O capacity depleted");
      success = false;
      break;
    }

    if (!simulate_adsorbent_bed(co2, ads1_capacity_, ads1_removal_, ads1_temp_limit_, "Adsorbent Bed 1", t_ads1)) {
      publish_bed_heartbeat("Adsorbent Bed 1", false, "CO₂ capacity depleted");
      success = false;
      break;
    }

    if (!simulate_adsorbent_bed(co2, ads2_capacity_, ads2_removal_, ads2_temp_limit_, "Adsorbent Bed 2", t_ads2)) {
      publish_bed_heartbeat("Adsorbent Bed 2", false, "CO₂ capacity depleted");
      success = false;
      break;
    }

    // VENTING
    if (co2 > 800.0) {
      feedback->venting = true;
      feedback->vent_bed_id = 3;
      feedback->vent_amount = co2 * 0.25;
      co2 -= feedback->vent_amount;
      total_vented += feedback->vent_amount;
      vents++;

      this->total_co2_storage_ += feedback->vent_amount;

      // CO₂ sensor failure simulation
      if (enable_failure_ && (rand() % 500) == 0) {
        RCLCPP_FATAL(this->get_logger(), "[FAILURE] CO₂ sensor failed. Cannot proceed.");
        publish_bed_heartbeat("Sensor", false, "CO₂ sensor failure");
        success = false;
        break;
      }

      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[VENT] CO₂ > 800ppm. Vented %.2f", feedback->vent_amount);
    } else {
      feedback->venting = false;
      feedback->vent_bed_id = -1;
      feedback->vent_amount = 0.0;
    }

    goal_handle->publish_feedback(feedback);
    rclcpp::sleep_for(100ms);
    cycles++;
  }

  result->success = success;
  result->cycles_completed = cycles;
  result->total_vents = vents;
  result->total_co2_vented = total_vented;
  result->summary_message = success ? "ARS simulation completed" : "Simulation terminated early";

  publish_bed_heartbeat("ARS", success, result->summary_message);
  goal_handle->succeed(result);
}


bool ARSActionServer::simulate_desiccant_bed(float &h2o, float cap, float rate, float max_temp, const std::string &name, float &temp)
{
  h2o -= rate;
  temp += 0.5;
  if (enable_failure_ && (rand() % 100) < 5) {  // 5% chance of failure
    RCLCPP_ERROR(this->get_logger(), "[FAILURE] %s failed to remove contaminants. TCCS malfunction!", name.c_str());
    publish_bed_heartbeat(name, false, "TCCS failure - contaminants leaked");
    return false;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "[%s] Removing H₂O %.2f, Temp: %.2f°C", name.c_str(), rate, temp);
  return h2o >= 0.0 && temp <= max_temp;
}

bool ARSActionServer::simulate_adsorbent_bed(float &co2, float cap, float rate, float max_temp, const std::string &name, float &temp)
{
  temp += 10.0;
  if (temp < 200.0) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "[%s] Heating... Temp: %.2f°C", name.c_str(), temp);
    return true;
  }

  co2 -= rate;
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "[%s] Removing CO₂ %.2f, Temp: %.2f°C", name.c_str(), rate, temp);
  return co2 >= 0.0 && temp <= max_temp;
}

void ARSActionServer::publish_bed_heartbeat(const std::string &bed, bool ok, const std::string &msg)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = bed + " Status";
  status.level = ok ? status.OK : status.ERROR;
  status.message = msg;
  status.hardware_id = "ARS_BED";
  heartbeat_pub_->publish(status);
}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node_options = rclcpp::NodeOptions();
  auto ars_node = std::make_shared<ARSActionServer>(node_options);
  rclcpp::spin(ars_node);
  rclcpp::shutdown();
  return 0;
}