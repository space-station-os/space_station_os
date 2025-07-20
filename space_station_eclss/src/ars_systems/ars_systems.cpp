#include "space_station_eclss/ars_systems.hpp"

namespace space_station_eclss
{

ARSActionServer::ARSActionServer() : Node("air_revitalisation")
{
  // Declare and get parameters
  this->declare_parameter("sim_step_duration", 0.1);
  this->declare_parameter("desiccant_capacity_1", 400.0);
  this->declare_parameter("desiccant_capacity_2", 400.0);
  this->declare_parameter("adsorbent_capacity_1", 850.0);
  this->declare_parameter("adsorbent_capacity_2", 850.0);
  this->declare_parameter("removal_efficiency", 0.95);
  this->declare_parameter("vent_ratio", 0.4);
  this->declare_parameter("initial_co2_level", 1000.0);
  this->declare_parameter("co2_generation_per_crew", 1.04);
  this->declare_parameter("crew_count", 4);
  this->declare_parameter("co2_tank_capacity", 500.0);
  this->get_parameter("co2_tank_capacity", co2_tank_capacity_);

  this->get_parameter("sim_step_duration", sim_step_duration_);
  this->get_parameter("desiccant_capacity_1", desiccant_capacity_1_);
  this->get_parameter("desiccant_capacity_2", desiccant_capacity_2_);
  this->get_parameter("adsorbent_capacity_1", adsorbent_capacity_1_);
  this->get_parameter("adsorbent_capacity_2", adsorbent_capacity_2_);
  this->get_parameter("removal_efficiency", removal_efficiency_);
  this->get_parameter("vent_ratio", vent_ratio_);
  this->get_parameter("initial_co2_level", initial_co2_level_);
  this->get_parameter("co2_generation_per_crew", co2_generation_per_crew_);
  this->get_parameter("crew_count", crew_count_);

  current_co2_level_ = initial_co2_level_;
  failure_triggered_ = false;
  cycle_counter_ = 0;
  total_vent_events_ = 0;
  total_co2_vented_ = 0.0;

  action_server_ = rclcpp_action::create_server<AirRevitalisation>(
    this,
    "air_revitalisation",
    std::bind(&ARSActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ARSActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ARSActionServer::handle_accepted, this, std::placeholders::_1));

  heartbeat_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("ars/heartbeat", 10);
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      if (!failure_triggered_) {
        send_heartbeat("OK", "ARS nominal operation");
      } else {
        send_heartbeat("ERROR", failure_message_);
      }
    });

  co2_storage = this->create_publisher<std_msgs::msg::Float64>("/co2_storage", 10);

  RCLCPP_INFO(this->get_logger(), "ARS Action Server Initialized");
}

rclcpp_action::GoalResponse ARSActionServer::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const AirRevitalisation::Goal>)
{
  RCLCPP_INFO(this->get_logger(), "[Goal] Received new ARS simulation goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ARSActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleAirRevitalisation>)
{
  RCLCPP_WARN(this->get_logger(), "[Cancel] ARS goal was requested to be cancelled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ARSActionServer::handle_accepted(const std::shared_ptr<GoalHandleAirRevitalisation> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "[Accepted] Executing ARS simulation...");
  std::thread{std::bind(&ARSActionServer::execute, this, goal_handle)}.detach();
}

void ARSActionServer::send_heartbeat(const std::string & level, const std::string & message)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "ARS Simulation Heartbeat";
  status.level = (level == "OK") ? status.OK : status.ERROR;
  status.message = message;
  heartbeat_pub_->publish(status);
}

void ARSActionServer::simulateDesiccantBed1(float &moisture_content, rclcpp::Time)
{
  moisture_content *= 0.8;
  RCLCPP_DEBUG(this->get_logger(), "Desiccant Bed 1 reduced moisture to %.2f", moisture_content);
}

void ARSActionServer::simulateDesiccantBed2(float &moisture_content, rclcpp::Time)
{
  moisture_content *= 1.05;
  RCLCPP_DEBUG(this->get_logger(), "Desiccant Bed 2 added humidity to %.2f", moisture_content);
}

void ARSActionServer::simulateAdsorbentBed1(
  float &co2_content, rclcpp::Time, bool &venting, int &vent_bed_id, float &vent_amount)
{
  float removed = co2_content * removal_efficiency_;
  co2_content -= removed;
  venting = true;
  vent_bed_id = 1;
  vent_amount = removed * vent_ratio_;
  total_vent_events_++;
  total_co2_vented_ += vent_amount;

  RCLCPP_INFO(this->get_logger(), "Adsorbent Bed 1 vented %.2f CO2", vent_amount);
}

void ARSActionServer::simulateAdsorbentBed2(
  float &co2_content, rclcpp::Time, bool &venting, int &vent_bed_id, float &vent_amount)
{
  co2_content *= 1.02;
  venting = true;
  vent_bed_id = 2;
  vent_amount = co2_content * 0.01;
  total_vent_events_++;
  total_co2_vented_ += vent_amount;

  RCLCPP_INFO(this->get_logger(), "Adsorbent Bed 2 desorbed and vented %.2f CO2", vent_amount);
}

void ARSActionServer::execute(const std::shared_ptr<GoalHandleAirRevitalisation> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<AirRevitalisation::Feedback>();
  auto result = std::make_shared<AirRevitalisation::Result>();

  rclcpp::Rate rate(1.0 / sim_step_duration_);
  int sim_time = goal->sim_time;
  float moisture = 50.0;

  RCLCPP_INFO(this->get_logger(), "ARS Simulation started for %d seconds", sim_time);

  for (int t = 0; t <= sim_time; ++t)
  {
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->summary_message = "Simulation cancelled by client.";
      goal_handle->canceled(result);
      RCLCPP_WARN(this->get_logger(), "ARS Simulation cancelled");
      return;
    }

    // Moisture management
    simulateDesiccantBed1(moisture, now());
    simulateDesiccantBed2(moisture, now());

    // CO2 generation from crew
    current_co2_level_ += crew_count_ * co2_generation_per_crew_;

    // Check for tank threshold
    bool venting = false;
    int vent_bed_id = 0;
    float vent_amount = 0.0;

    if (current_co2_level_ >= co2_tank_capacity_) {
      venting = true;
      vent_bed_id = 1;  // Assume venting via Bed 1
      vent_amount = current_co2_level_ * vent_ratio_;
      current_co2_level_ = 0.0;  // Reset CO2 after venting

      std_msgs::msg::Float64 co2_msg;
      co2_msg.data = vent_amount;
      co2_storage->publish(co2_msg);

      total_vent_events_++;
      total_co2_vented_ += vent_amount;

      RCLCPP_INFO(this->get_logger(),
                  "[VENT] CO₂ storage threshold reached. Vented %.2f units to Sabatier tank",
                  vent_amount);
    }

    // Feedback
    feedback->current_time = t;
    feedback->cabin_co2_level = current_co2_level_;
    feedback->venting = venting;
    feedback->vent_bed_id = vent_bed_id;
    feedback->vent_amount = vent_amount;
    goal_handle->publish_feedback(feedback);

    // Simulate failure at halfway point if enabled
    if (goal->simulate_failure && t == sim_time / 2) {
      failure_triggered_ = true;
      failure_message_ = "Simulated failure: Adsorbent Bed 1 stuck closed.";
      RCLCPP_ERROR(this->get_logger(), "[FAILURE] %s", failure_message_.c_str());
    }

    rate.sleep();
  }

  // Final result
  cycle_counter_++;
  result->success = !failure_triggered_;
  result->cycles_completed = cycle_counter_;
  result->total_vents = total_vent_events_;
  result->total_co2_vented = total_co2_vented_;
  result->summary_message = failure_triggered_ ? failure_message_ : "ARS simulation completed successfully";

  goal_handle->succeed(result);

  RCLCPP_INFO(this->get_logger(),
              "[COMPLETE] Simulation ended. Success: %s | Vents: %d | Total CO₂ Vented: %.2f",
              result->success ? "YES" : "NO",
              result->total_vents,
              result->total_co2_vented);
}

}  // namespace space_station_eclss

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<space_station_eclss::ARSActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
