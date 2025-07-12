#ifndef ARS_ACTION_SERVER_HPP_
#define ARS_ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <space_station_eclss/action/air_revitalisation.hpp>

namespace space_station_eclss {

class ARSActionServer : public rclcpp::Node
{
public:
  using AirRevitalisation = space_station_eclss::action::AirRevitalisation;
  using GoalHandleARS = rclcpp_action::ServerGoalHandle<AirRevitalisation>;


  explicit ARSActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void execute(const std::shared_ptr<GoalHandleARS> goal_handle);

  bool simulate_desiccant_bed(float &h2o, float cap, float rate, float max_temp, const std::string &name, float &temp);
  bool simulate_adsorbent_bed(float &co2, float cap, float rate, float max_temp, const std::string &name, float &temp);
  void publish_bed_heartbeat(const std::string &bed, bool ok, const std::string &msg);

  rclcpp_action::Server<AirRevitalisation>::SharedPtr action_server_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr co2_storage_pub_;


  rclcpp::TimerBase::SharedPtr co2_storage_timer_;
  double total_co2_storage_ = 0.0;  
  int sim_time_;

  bool enable_failure_;
  double max_co2_storage_;

  float des1_capacity_, des1_removal_, des1_temp_limit_;
  float des2_capacity_, des2_removal_, des2_temp_limit_;
  float ads1_capacity_, ads1_removal_, ads1_temp_limit_;
  float ads2_capacity_, ads2_removal_, ads2_temp_limit_;
};

} 

#endif  // ARS_ACTION_SERVER_HPP_
