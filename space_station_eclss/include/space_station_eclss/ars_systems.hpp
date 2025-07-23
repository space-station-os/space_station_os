#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include "space_station_eclss/action/air_revitalisation.hpp"
#include "space_station_eclss/srv/co2_request.hpp"

namespace space_station_eclss {

class ARSActionServer : public rclcpp::Node {
public:
  using AirRevitalisation = space_station_eclss::action::AirRevitalisation;
  using GoalHandleARS = rclcpp_action::ServerGoalHandle<AirRevitalisation>;
  using Co2Request = space_station_eclss::srv::Co2Request;

  explicit ARSActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void execute(const std::shared_ptr<GoalHandleARS> goal_handle);
  void publish_bed_heartbeat(const std::string &bed, bool ok, const std::string &msg);
  bool simulate_desiccant_bed(float &h2o, float cap, float rate, float max_temp, const std::string &name, float &temp);
  bool simulate_adsorbent_bed(float &co2, float cap, float rate, float max_temp, const std::string &name, float &temp);
  void monitor_combustion_and_contaminants();
  void handle_co2_service(const std::shared_ptr<Co2Request::Request> req,
                          std::shared_ptr<Co2Request::Response> res);
  rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const AirRevitalisation::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<AirRevitalisation>> goal_handle);
  void simulate_co2_leak();
  rclcpp_action::Server<AirRevitalisation>::SharedPtr action_server_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr co2_storage_pub_;
  rclcpp::Service<Co2Request>::SharedPtr co2_request_srv_;
  rclcpp::TimerBase::SharedPtr co2_pub_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr combustion_timer_;
  rclcpp::TimerBase::SharedPtr contaminant_timer_;
  rclcpp::TimerBase::SharedPtr co2_leak_timer_;

  // Simulation Parameters
  int sim_time_;
  bool enable_failure_;
  float max_co2_storage_;
  float total_co2_storage_ = 0.0;
  float contaminant_level_ = 0.0;
  float contaminant_limit_ = 100.0; // ppm threshold
  double co2_leak_rate_;
  double co2_leak_threshold_;
  double last_co2_level_ = 0.0;

  // Desiccant Beds
  float des1_capacity_, des1_removal_, des1_temp_limit_;
  float des2_capacity_, des2_removal_, des2_temp_limit_;

  // Adsorbent Beds
  float ads1_capacity_, ads1_removal_, ads1_temp_limit_;
  float ads2_capacity_, ads2_removal_, ads2_temp_limit_;
};

}  // namespace space_station_eclss
