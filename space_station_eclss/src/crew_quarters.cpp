#include "space_station_eclss/crew_quarter.hpp"

HumanSimulationNode::HumanSimulationNode()
: Node("crew_quarters_node")
{
  this->declare_parameter<int>("crew_size", 4);
  this->declare_parameter<int>("events_per_day", 7);
  this->declare_parameter<int>("number_of_days", 5);
  this->declare_parameter<std::string>("mode", "rest");
  this->declare_parameter<float>("calorie_intake", 2000.0);
  this->declare_parameter<float>("potable_water_intake", 2.5);

  this->get_parameter("crew_size", crew_size_);
  this->get_parameter("events_per_day", events_per_day_);
  this->get_parameter("number_of_days", number_of_days_);
  this->get_parameter("mode", mode_);
  this->get_parameter("calorie_intake", calorie_intake_);
  this->get_parameter("potable_water_intake", potable_water_intake_);

  ars_client_ = rclcpp_action::create_client<space_station_eclss::action::AirRevitalisation>(this, "air_revitalisation");
  ogs_client_ = rclcpp_action::create_client<space_station_eclss::action::OxygenGeneration>(this, "oxygen_generation");
  wrs_client_ = rclcpp_action::create_client<space_station_eclss::action::WaterRecovery>(this, "water_recovery_systems");

  o2_service_client_ = this->create_client<space_station_eclss::srv::O2Request>("/ogs/request_o2");
  water_service_client_ = this->create_client<space_station_eclss::srv::RequestProductWater>("/wrs/product_water_request");

  int retries = 0;
  while (rclcpp::ok() && retries < 30 &&
        (!ars_client_->wait_for_action_server(std::chrono::seconds(1)) ||
         !ogs_client_->wait_for_action_server(std::chrono::seconds(1)) ||
         !wrs_client_->wait_for_action_server(std::chrono::seconds(1)))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for ARS, OGS, and WRS action servers... (%d)", retries);
    retries++;
  }

  if (retries >= 30) {
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for action servers. Exiting.");
    return;
  }

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
    return;
  }

  float o2_rest = 0.771f;
  float o2_exercise = 0.376f;
  float o2_total_per_person = (mode_ == "exercise") ? (o2_rest + o2_exercise) : o2_rest;
  float total_o2 = o2_total_per_person * crew_size_;

  float co2_per_person = (calorie_intake_ >= 2000.0f) ? 350.0f :
                         (calorie_intake_ >= 1500.0f) ? 262.0f : 175.0f;
  float total_co2 = co2_per_person * crew_size_;

  float total_water = potable_water_intake_ * crew_size_;
  float total_urine = 1.0f * events_per_day_ * crew_size_;

  send_o2_request(total_o2);
  send_water_request(total_water);
  send_ars_goal(total_co2, 0.8f * total_water);
  send_ogs_goal(total_water);
  send_wrs_goal(total_urine);

  current_event_count_++;
  RCLCPP_INFO(this->get_logger(), "Event %d of Day %d complete", current_event_count_, current_day_ + 1);

  if (current_event_count_ >= events_per_day_) {
    current_day_++;
    current_event_count_ = 0;
    RCLCPP_INFO(this->get_logger(), "=== Completed Day %d ===", current_day_);
  }
}


void HumanSimulationNode::send_o2_request(float o2_amount)
{
  if (!o2_service_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "O2 service not available.");
    return;
  }

  auto request = std::make_shared<space_station_eclss::srv::O2Request::Request>();
  request->o2_req = o2_amount;

  o2_service_client_->async_send_request(request,
    [this](std::shared_future<space_station_eclss::srv::O2Request::Response::SharedPtr> future) {
      if (future.valid()) {
        auto response = future.get();
        if (response) { // Check if the response pointer is valid
          if (!response->success) {
            RCLCPP_WARN(this->get_logger(), "O2 request denied: %s", response->message.c_str());
          } else {
            RCLCPP_INFO(this->get_logger(), "O2 request successful.");
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Received null O2 service response.");
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call O2 service: future not valid.");
      }
    });
}

void HumanSimulationNode::send_water_request(float water_amount)
{
  if (!water_service_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "Water service not available.");
    return;
  }

  auto request = std::make_shared<space_station_eclss::srv::RequestProductWater::Request>();
  request->amount = water_amount;

  water_service_client_->async_send_request(request,
    [this](std::shared_future<space_station_eclss::srv::RequestProductWater::Response::SharedPtr> future) {
      if (future.valid()) {
        auto response = future.get();
        if (response) { // Check if the response pointer is valid
          if (!response->success) {
            RCLCPP_WARN(this->get_logger(), "Water request denied: %s", response->message.c_str());
          } else {
            RCLCPP_INFO(this->get_logger(), "Water request successful.");
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Received null Water service response.");
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call Water service: future not valid.");
      }
    });
}



void HumanSimulationNode::send_ars_goal(float co2_mass, float moisture_content)
{
  auto goal = space_station_eclss::action::AirRevitalisation::Goal();
  goal.initial_co2_mass = co2_mass;
  goal.initial_moisture_content = moisture_content;
  goal.initial_contaminants = 5.0f;
  ars_client_->async_send_goal(goal);
}

void HumanSimulationNode::send_ogs_goal(float water_mass)
{
  auto goal = space_station_eclss::action::OxygenGeneration::Goal();
  goal.input_water_mass = water_mass;
  ogs_client_->async_send_goal(goal);
}

void HumanSimulationNode::send_wrs_goal(float urine_volume)
{
  auto goal = space_station_eclss::action::WaterRecovery::Goal();
  goal.urine_volume = urine_volume;
  wrs_client_->async_send_goal(goal);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HumanSimulationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
