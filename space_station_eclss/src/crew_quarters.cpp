#include "space_station_eclss/crew_quarter.hpp"

HumanSimulationNode::HumanSimulationNode()
: Node("crew_quarters_node"),
  current_day_(0),
  events_completed_today_(0),
  ars_sent_(false),
  wrs_sent_(false),
  ogs_sent_(false),
  current_o2_reserve_(0.0),
  current_water_reserve_(0.0)
{
  this->declare_parameter<int>("crew_size", 4);
  this->declare_parameter<int>("events_per_day", 7);
  this->declare_parameter<int>("number_of_days", 1);
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
  wrs_client_ = rclcpp_action::create_client<space_station_eclss::action::WaterRecovery>(this, "water_recovery_systems");
  o2_service_client_ = this->create_client<space_station_eclss::srv::O2Request>("ogs/request_o2");
  water_service_client_ = this->create_client<space_station_eclss::srv::RequestProductWater>("wrs/product_water_request");

  while (!ars_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for ARS action server...");
  }
  while (!wrs_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for WRS action server...");
  }

  o2_storage_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "o2_storage", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      current_o2_reserve_ = msg->data;
    });

  water_reserve_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "wrs/product_water_reserve", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      current_water_reserve_ = msg->data;
    });

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&HumanSimulationNode::simulate_event, this));
}

void HumanSimulationNode::simulate_event()
{
  if (current_day_ >= number_of_days_) {
    RCLCPP_INFO(this->get_logger(), "Simulation complete. Shutting down.");
    current_day_ = 0;
    events_completed_today_ = 0;
  }

  double o2_needed = 0.771 * crew_size_;
  if (mode_ == "exercise") {
    o2_needed += 0.376 * crew_size_;
  }
  float water_needed = potable_water_intake_ * crew_size_;

  if (!ars_sent_) {
    send_ars_goal();
    ars_sent_ = true;
  }

  if (!wrs_sent_ && current_water_reserve_ >= water_needed) {
    send_wrs_goal();
    request_water(water_needed);
    wrs_sent_ = true;
  } else if (!wrs_sent_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for water reserve. Water: %.2f / %.2f", current_water_reserve_, water_needed);
  }

  if (!ogs_sent_ && current_o2_reserve_ >= o2_needed) {
    request_o2(o2_needed);
    ogs_sent_ = true;
  } else if (!ogs_sent_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for O2 reserve. O2: %.2f / %.2f", current_o2_reserve_, o2_needed);
  }

  if (ars_sent_ && wrs_sent_ && ogs_sent_) {
    events_completed_today_++;
    ars_sent_ = wrs_sent_ = ogs_sent_ = false;

    if (events_completed_today_ >= events_per_day_) {
      current_day_++;
      events_completed_today_ = 0;
      RCLCPP_INFO(this->get_logger(), "Completed Day %d of %d", current_day_, number_of_days_);
    }
  }
}

void HumanSimulationNode::request_o2(double o2_required)
{
  auto req = std::make_shared<space_station_eclss::srv::O2Request::Request>();
  req->o2_req = o2_required;

  if (!o2_service_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "O2 service unavailable.");
    return;
  }

  auto future = o2_service_client_->async_send_request(
    req,
    [this](rclcpp::Client<space_station_eclss::srv::O2Request>::SharedFuture future_result) {
      auto res = future_result.get();
      if (res->success) {
        RCLCPP_INFO(this->get_logger(), "O2 granted: %.2f g", res->o2_resp);
      } else {
        RCLCPP_ERROR(this->get_logger(), "O2 request failed: %s", res->message.c_str());
      }
    }
  );

}

void HumanSimulationNode::request_water(float water_required)
{
  auto req = std::make_shared<space_station_eclss::srv::RequestProductWater::Request>();
  req->amount = water_required;

  if (!water_service_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Water service unavailable.");
    return;
  }

  auto future = water_service_client_->async_send_request(
    req,
    [this](rclcpp::Client<space_station_eclss::srv::RequestProductWater>::SharedFuture future_result) {
      auto res = future_result.get();
      if (res->success) {
        RCLCPP_INFO(this->get_logger(), "Water granted: %.2f L", res->water_granted);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Water request failed: %s", res->message.c_str());
      }
    }
  );

}

void HumanSimulationNode::send_ars_goal()
{
  float co2_per_person = (calorie_intake_ >= 2000.0f) ? 350.0f :
                         (calorie_intake_ >= 1500.0f) ? 262.0f : 175.0f;
  float total_co2 = co2_per_person * crew_size_;
  float total_water = potable_water_intake_ * crew_size_;
  float moisture_content = 0.8f * total_water;

  auto goal_msg = space_station_eclss::action::AirRevitalisation::Goal();
  goal_msg.initial_co2_mass = total_co2;
  goal_msg.initial_moisture_content = moisture_content;
  goal_msg.initial_contaminants = 5.0f;

  auto options = rclcpp_action::Client<space_station_eclss::action::AirRevitalisation>::SendGoalOptions();
  options.result_callback = [this](const auto &result) {
    if (result.result->success) {
      RCLCPP_INFO(this->get_logger(), "ARS Success: Vented %.2f ppm CO2 in %d cycles (%d vents)",
                  result.result->total_co2_vented,
                  result.result->cycles_completed,
                  result.result->total_vents);
    } else {
      RCLCPP_ERROR(this->get_logger(), "ARS Failed: %s", result.result->summary_message.c_str());
    }
  };

  ars_client_->async_send_goal(goal_msg, options);
}

void HumanSimulationNode::send_wrs_goal()
{
  float urine_volume = 1.0f * events_per_day_ * crew_size_;

  auto goal_msg = space_station_eclss::action::WaterRecovery::Goal();
  goal_msg.urine_volume = urine_volume;

  auto options = rclcpp_action::Client<space_station_eclss::action::WaterRecovery>::SendGoalOptions();
  options.result_callback = [this](const auto &result) {
    if (result.result->success) {
      RCLCPP_INFO(this->get_logger(), "WRS Success: Recovered %.2f L in %d cycles",
                  result.result->total_purified_water,
                  result.result->total_cycles);
    } else {
      RCLCPP_ERROR(this->get_logger(), "WRS Failed: %s", result.result->summary_message.c_str());
    }
  };

  wrs_client_->async_send_goal(goal_msg, options);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HumanSimulationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
