#ifndef SPACE_STATION_ECLSS_OGS_SYSTEM_HPP
#define SPACE_STATION_ECLSS_OGS_SYSTEM_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include "space_station_eclss/action/oxygen_generation.hpp"
#include "space_station_eclss/srv/co_request.hpp"
#include "space_station_eclss/srv/o2_request.hpp"

namespace space_station_eclss
{

class OGSActionServer : public rclcpp::Node
{
public:
  explicit OGSActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using OxygenGeneration = space_station_eclss::action::OxygenGeneration;
  using GoalHandleOGS = rclcpp_action::ServerGoalHandle<OxygenGeneration>;

  void declare_parameters();
  void co2_callback(const std_msgs::msg::Float64::SharedPtr msg);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const OxygenGeneration::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleOGS> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleOGS> goal_handle);
  void execute(const std::shared_ptr<GoalHandleOGS> goal_handle);

  float simulate_deionization(float iodine_ppm, float water_mass);
  std::pair<float, float> simulate_electrolysis(float water_mass);
  std::pair<float, float> simulate_sabatier(float h2_mass, float co2_mass);

  void publish_heartbeat(const std::string &msg, int level);
  void standby_publish();
  bool check_environmental_limits(float o2_partial_pressure, float total_pressure, float pressure_rate);

  rclcpp_action::Server<OxygenGeneration>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr co2_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr o2_pub_, methane_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr heartbeat_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr grey_water_client_;
  rclcpp::Client<space_station_eclss::srv::CoRequest>::SharedPtr co2_request_client_;
  rclcpp::Service<space_station_eclss::srv::O2Request>::SharedPtr o2_service_server_;

  rclcpp::TimerBase::SharedPtr standby_timer_;

  float current_co2_ = 0.0;
  float latest_o2_ = 0.0;
  float latest_ch4_ = 0.0;

  // Parameters
  bool enable_failure_;
  float electrolysis_temp_;
  float o2_efficiency_;
  float sabatier_efficiency_;
  float sabatier_temp_;
  float sabatier_pressure_;
};

}  // namespace space_station_eclss

#endif  // SPACE_STATION_ECLSS_OGS_SYSTEM_HPP
