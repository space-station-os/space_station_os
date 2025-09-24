#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_msgs/msg/bool.hpp>

#include "space_station_eclss/action/air_revitalisation.hpp"
#include "space_station_eclss/srv/co2_request.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// BehaviorTree.CPP v3.x
#include <behaviortree_cpp_v3/bt_factory.h>

namespace space_station_eclss
{

class ARSActionServer : public rclcpp::Node
{
public:
  using AirRevitalisation = space_station_eclss::action::AirRevitalisation;
  using GoalHandleARS = rclcpp_action::ServerGoalHandle<AirRevitalisation>;
  using Co2Request = space_station_eclss::srv::Co2Request;

  explicit ARSActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Utility getter so tests can check failure flag
  bool get_failure_enabled() const { return enable_failure_; }

private:
  // ---- ROS callbacks ----
  void declare_parameters();
  void execute(const std::shared_ptr<GoalHandleARS> goal_handle);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const AirRevitalisation::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<AirRevitalisation>> goal_handle);

  void handle_co2_service(
    const std::shared_ptr<Co2Request::Request> req,
    std::shared_ptr<Co2Request::Response> res);

  void monitor_combustion_and_contaminants();

  // ---- Simulation helpers ----
  void publish_bed_heartbeat(const std::string &bed, bool ok,
                             const std::string &msg,
                             const std::string &hardware_id = "ARS");

  bool simulate_desiccant_bed(float &h2o, float cap, float rate, float max_temp,
                              const std::string &name, float &temp);

  bool simulate_adsorbent_bed(float &co2, float cap, float rate, float max_temp,
                              const std::string &name, float &temp);

  // ---- ROS entities ----
  rclcpp_action::Server<AirRevitalisation>::SharedPtr action_server_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr co2_storage_pub_;
  rclcpp::Service<Co2Request>::SharedPtr co2_request_srv_;
  rclcpp::TimerBase::SharedPtr co2_pub_timer_;
  rclcpp::TimerBase::SharedPtr combustion_timer_;
  rclcpp::TimerBase::SharedPtr contaminant_timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr disable_failure_;

  // ---- Parameters & State ----
  int sim_time_{10};
  bool enable_failure_{true};
  float max_co2_storage_{3947.4};
  float total_co2_storage_{0.0};
  float contaminant_level_{0.0};
  float contaminant_limit_{100.0};

  // Bed parameters
  float des1_capacity_, des1_removal_, des1_temp_limit_;
  float des2_capacity_, des2_removal_, des2_temp_limit_;
  float ads1_capacity_, ads1_removal_, ads1_temp_limit_;
  float ads2_capacity_, ads2_removal_, ads2_temp_limit_;
};

}  // namespace space_station_eclss
