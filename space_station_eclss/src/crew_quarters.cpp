#include "space_station_eclss/crew_quarter.hpp"

HumanSimulationNode::HumanSimulationNode()
: Node("crew_quarters_node")
{
  // Declare parameters
  this->declare_parameter<int>("crew_size", 4);
  this->declare_parameter<int>("events_per_day", 7);
  this->declare_parameter<int>("number_of_days", 5);
  this->declare_parameter<std::string>("mode", "rest");
  this->declare_parameter<float>("calorie_intake", 2000.0);
  this->declare_parameter<float>("potable_water_intake", 2.5);

  // Get parameter values
  this->get_parameter("crew_size", crew_size_);
  this->get_parameter("events_per_day", events_per_day_);
  this->get_parameter("number_of_days", number_of_days_);
  this->get_parameter("mode", mode_);
  this->get_parameter("calorie_intake", calorie_intake_);
  this->get_parameter("potable_water_intake", potable_water_intake_);

  // Action Clients
  ars_client_ = rclcpp_action::create_client<space_station_eclss::action::AirRevitalisation>(this, "air_revitalisation");
  ogs_client_ = rclcpp_action::create_client<space_station_eclss::action::OxygenGeneration>(this, "oxygen_generation");
  wrs_client_ = rclcpp_action::create_client<space_station_eclss::action::WaterRecovery>(this, "water_recovery_systems");

  // Service Clients
  o2_service_client_ = this->create_client<space_station_eclss::srv::O2Request>("/ogs/request_o2");
  water_service_client_ = this->create_client<space_station_eclss::srv::RequestProductWater>("/wrs/product_water_request");

  int retries = 0;
  while (rclcpp::ok() && retries < 30 &&
      (!ars_client_->wait_for_action_server(std::chrono::seconds(1)) ||
       !ogs_client_->wait_for_action_server(std::chrono::seconds(1)) ||
       !wrs_client_->wait_for_action_server(std::chrono::seconds(1))))
  {
    RCLCPP_WARN(this->get_logger(), "Waiting for ARS, OGS, and WRS action servers... (%d)", retries);
    retries++;
  }

  if (retries >= 30) {
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for action servers. Exiting.");
    rclcpp::shutdown();
  }

  // Timer to simulate metabolic cycles
  simulation_timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    std::bind(&HumanSimulationNode::simulate_metabolic_cycle, this)
  );

  RCLCPP_INFO(this->get_logger(), "Human Simulation Node started.");
}

void HumanSimulationNode::simulate_metabolic_cycle()
{
  if (current_day_ >= number_of_days_) {
    RCLCPP_INFO(this->get_logger(), "Simulation complete: %d days done.", current_day_);
    rclcpp::shutdown();
    return;
  }

  // === 1. Oxygen Consumption ===
  float o2_rest = 0.771f;
  float o2_exercise = 0.376f;
  float o2_total_per_person = (mode_ == "exercise") ? (o2_rest + o2_exercise) : o2_rest;
  float total_o2 = o2_total_per_person * crew_size_;

  // === 2. CO2 Production ===
  float co2_per_person;
  if (calorie_intake_ >= 2000.0f)
    co2_per_person = 350.0f;
  else if (calorie_intake_ >= 1500.0f)
    co2_per_person = 262.0f;
  else
    co2_per_person = 175.0f;

  float total_co2 = co2_per_person * crew_size_;

  // === 3. Water Intake ===
  float total_water = potable_water_intake_ * crew_size_;

  // === 4. Waste ===
  float total_urine = 1.0f * events_per_day_ * crew_size_;
  float total_feces = 0.5f * 2 * crew_size_;  // Not used currently

  // === 5. Call O2 Service ===
  auto o2_request = std::make_shared<space_station_eclss::srv::O2Request::Request>();
  o2_request->o2_req = total_o2;

  if (!o2_service_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "O2 service not available.");
    return;
  }

  auto o2_future = o2_service_client_->async_send_request(o2_request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), o2_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call O2 service.");
    return;
  }

  auto o2_response = o2_future.get();
  if (!o2_response->success) {
    RCLCPP_WARN(this->get_logger(), "O2 request denied: %s", o2_response->message.c_str());
    return;
  }

  // === 6. Call Water Service ===
  auto water_request = std::make_shared<space_station_eclss::srv::RequestProductWater::Request>();
  water_request->amount = total_water;

  if (!water_service_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "Water service not available.");
    return;
  }

  auto water_future = water_service_client_->async_send_request(water_request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), water_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call water service.");
    return;
  }

  auto water_response = water_future.get();
  if (!water_response->success) {
    RCLCPP_WARN(this->get_logger(), "Water request denied: %s", water_response->message.c_str());
    return;
  }

  // === 7. ARS Goal ===
  auto ars_goal = space_station_eclss::action::AirRevitalisation::Goal();
  ars_goal.initial_co2_mass = total_co2;
  ars_goal.initial_moisture_content = 0.8f * total_water;
  ars_goal.initial_contaminants = 5.0f;

  ars_client_->async_send_goal(ars_goal);

  // === 8. OGS Goal ===
  auto ogs_goal = space_station_eclss::action::OxygenGeneration::Goal();
  ogs_goal.input_water_mass = total_water;

  ogs_client_->async_send_goal(ogs_goal);

  // === 9. WRS Goal ===
  auto wrs_goal = space_station_eclss::action::WaterRecovery::Goal();
  wrs_goal.urine_volume = total_urine;

  wrs_client_->async_send_goal(wrs_goal);

  // === 10. Logging ===
  current_event_count_++;
  RCLCPP_INFO(this->get_logger(), "Event %d of Day %d complete", current_event_count_, current_day_ + 1);

  if (current_event_count_ >= events_per_day_) {
    current_day_++;
    current_event_count_ = 0;
    RCLCPP_INFO(this->get_logger(), "=== Completed Day %d ===", current_day_);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HumanSimulationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
