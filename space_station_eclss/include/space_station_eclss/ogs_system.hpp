#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic>
#include "space_station_interfaces/action/oxygen_generation.hpp"
#include "space_station_interfaces/srv/request_product_water.hpp"
#include "space_station_interfaces/srv/co2_request.hpp"
#include "space_station_interfaces/srv/o2_request.hpp"
#include "space_station_interfaces/srv/grey_water.hpp"
#include "space_station_interfaces/srv/load.hpp"
// BehaviorTree
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
namespace space_station_eclss{

using namespace std::chrono_literals;
class OGSSystem : public rclcpp::Node
{
public:
  using OxygenGeneration = space_station_interfaces::action::OxygenGeneration;
  using GoalHandleOGS = rclcpp_action::ServerGoalHandle<OxygenGeneration>;

  explicit OGSSystem(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("Water_in"),
      BT::OutputPort<double>("O2_out"),
      BT::OutputPort<double>("H2_out"),
      BT::InputPort<double>("CO2_in"),
      BT::InputPort<double>("H2_in"),
      BT::OutputPort<double>("Water_out"),
      BT::OutputPort<double>("GreyWater_out")
    };
  }

private:
  // Parameters
  bool enable_failure_;
  double electrolysis_temp_;
  double o2_efficiency_;
  double sabatier_efficiency_;
  double sabatier_temp_;
  double sabatier_pressure_;
  double min_o2_capacity_;
  double max_o2_capacity_;

  bool powered_;
  double latest_o2_ = 0.0;
  double total_ch4_vented_ = 0.0;
  double last_h2_generated_ = 0.0;
  double last_o2_generated_ = 0.0;
  double h2_generated = 0.0; 
  // ROS entities
  rclcpp_action::Server<OxygenGeneration>::SharedPtr action_server_;
  rclcpp::Client<space_station_interfaces::srv::RequestProductWater>::SharedPtr water_client_;
  rclcpp::Client<space_station_interfaces::srv::Co2Request>::SharedPtr co2_client_;
  rclcpp::Client<space_station_interfaces::srv::GreyWater>::SharedPtr gray_water_client_;
  rclcpp::Service<space_station_interfaces::srv::O2Request>::SharedPtr o2_server_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr co2_storage_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr o2_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ch4_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::Client<space_station_interfaces::srv::Load>::SharedPtr load_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr power_retry_timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr disable_failure_;
  std_msgs::msg::Float64 ch4_msg;
  // Callbacks
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &,
                                          std::shared_ptr<const OxygenGeneration::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleOGS> goal_handle);
  void execute_goal(const std::shared_ptr<GoalHandleOGS> goal_handle);

  void o2_service_callback(
    const std::shared_ptr<space_station_interfaces::srv::O2Request::Request> request,
    std::shared_ptr<space_station_interfaces::srv::O2Request::Response> response);

  void publish_failure_diagnostics(const std::string &unit, const std::string &reason);
  void publish_periodic_status();
  void initialize_systems();
  bool supply_load();
  // Support functions (same as your current ones)
  void request_product_water(double amount_liters);
  void request_co2(double co2_mass_kg);
  void send_gray_water(double amount_liters);
  
  // === BehaviorTree XML path ===
  std::string bt_xml_file_;
  rclcpp::Time last_co2_request_time_;
  std::atomic<double> co2_storage_{0.0};
};
}
