#ifndef OGS_ACTION_SERVER_HPP_
#define OGS_ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include "space_station_eclss/action/oxygen_generation.hpp"

namespace space_station_eclss {

class OGSActionServer : public rclcpp::Node
{
public:
  using OxygenGeneration = space_station_eclss::action::OxygenGeneration;
  using GoalHandleOGS = rclcpp_action::ServerGoalHandle<OxygenGeneration>;

  explicit OGSActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void execute(const std::shared_ptr<GoalHandleOGS> goal_handle);

  // Subsystem simulations
  float simulate_deionization(float iodine_ppm, float water_mass);
  std::pair<float, float> simulate_electrolysis(float water_mass);
  std::pair<float, float> simulate_sabatier(float h2_mass, float co2_mass);

  // CO2 input
  void co2_callback(const std_msgs::msg::Float64::SharedPtr msg);

  // Action interfaces
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const OxygenGeneration::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleOGS> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleOGS> goal_handle);

  // Heartbeat + data publishing
  void publish_heartbeat(const std::string &msg, int level);
  void standby_publish();

  // Timers
  rclcpp::TimerBase::SharedPtr standby_timer_;

  // Action server and comms
  rclcpp_action::Server<OxygenGeneration>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr o2_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr methane_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr heartbeat_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr grey_water_client_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr co2_sub_;

  // Parameters
  float electrolysis_temp_;
  float o2_efficiency_;
  float sabatier_efficiency_;
  float sabatier_temp_;
  float sabatier_pressure_;
  bool enable_failure_;

  // State
  float current_co2_ = 0.0;
  float latest_o2_ = 0.0;
  float latest_ch4_ = 0.0;
};

}  // namespace space_station_eclss

#endif  // OGS_ACTION_SERVER_HPP_
