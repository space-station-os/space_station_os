#ifndef SPACE_STATION_ECLSS__ARS_ACTION_SERVER_HPP_
#define SPACE_STATION_ECLSS__ARS_ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_msgs/msg/float64.hpp>
#include "space_station_eclss/action/air_revitalisation.hpp"

namespace space_station_eclss
{

class ARSActionServer : public rclcpp::Node
{
public:
  using AirRevitalisation = space_station_eclss::action::AirRevitalisation;
  using GoalHandleAirRevitalisation = rclcpp_action::ServerGoalHandle<AirRevitalisation>;

  ARSActionServer();

private:
  rclcpp_action::Server<AirRevitalisation>::SharedPtr action_server_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr co2_storage;

  // Parameters
  double sim_step_duration_;
  double desiccant_capacity_1_;
  double desiccant_capacity_2_;
  double adsorbent_capacity_1_;
  double adsorbent_capacity_2_;
  double removal_efficiency_;
  double vent_ratio_;
  double initial_co2_level_;
  double co2_generation_per_crew_;
  int crew_count_;

  // Internal state
  float current_co2_level_;
  bool failure_triggered_;
  std::string failure_message_;
  int cycle_counter_;
  int total_vent_events_;
  float total_co2_vented_;
  double co2_tank_capacity_;

  void send_heartbeat(const std::string & level, const std::string & message);
  void simulateDesiccantBed1(float &moisture_content, rclcpp::Time time);
  void simulateDesiccantBed2(float &moisture_content, rclcpp::Time time);
  void simulateAdsorbentBed1(float &co2_content, rclcpp::Time time, bool &venting, int &vent_bed_id, float &vent_amount);
  void simulateAdsorbentBed2(float &co2_content, rclcpp::Time time, bool &venting, int &vent_bed_id, float &vent_amount);

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const AirRevitalisation::Goal>);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAirRevitalisation>);
  void handle_accepted(const std::shared_ptr<GoalHandleAirRevitalisation>);

  void execute(const std::shared_ptr<GoalHandleAirRevitalisation> goal_handle);
};

}  // namespace space_station_eclss

#endif  // SPACE_STATION_ECLSS__ARS_ACTION_SERVER_HPP_
