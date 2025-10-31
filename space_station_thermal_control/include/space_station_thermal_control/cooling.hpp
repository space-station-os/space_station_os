#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>


#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <space_station_interfaces/srv/request_product_water.hpp>
#include <space_station_interfaces/action/coolant.hpp>
#include <space_station_interfaces/srv/vent_heat.hpp>
#include <space_station_interfaces/msg/external_loop_status.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>


namespace space_station_thermal_control
{

class CoolantActionServer : public rclcpp::Node
{
public:
  using Coolant = space_station_interfaces::action::Coolant;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Coolant>;

  explicit CoolantActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // -------------------- Parameters --------------------
  double mass_kg_{200.0};
  double cp_water_{4186.0};
  double transfer_efficiency_{0.85};
  double vent_threshold_kj_{250.0};
  bool diagnostics_enabled_{true};
  int goal_counter_{0};
  double coolant_temp_c_{25.0};
  double lasted_vented_heat_{0.0};

  // -------------------- ROS Interfaces --------------------
  rclcpp_action::Server<Coolant>::SharedPtr action_server_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr internal_loop_pub_;
  rclcpp::Publisher<space_station_interfaces::msg::ExternalLoopStatus>::SharedPtr external_loop_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;

  // Clients
  rclcpp::Client<space_station_interfaces::srv::RequestProductWater>::SharedPtr product_water_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr grey_water_client_;
  rclcpp::Client<space_station_interfaces::srv::VentHeat>::SharedPtr radiator_client_;

  // Timer for BT tick
  rclcpp::TimerBase::SharedPtr timer_;

  // -------------------- Core Methods --------------------
  void recycleWater();
  void publishInternalLoop();
  void publishExternalLoop(double received_heat_kj);
  void publishDiagnostics(bool status, const std::string &message);

  // Action server handlers
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid,
                                         std::shared_ptr<const Coolant::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);



  // -------------------- Behavior Tree Integration --------------------
  BT::Tree tree_;

  // BT tick loop
  void tickBehaviorTree();

  // --- Behavior Tree leaf nodes ---
  BT::NodeStatus isTempHigh();
  BT::NodeStatus isAmmoniaHot();
  BT::NodeStatus ventHeat();
  BT::NodeStatus coolLoop();
  BT::NodeStatus refreshWater();

  // --- Behavior Tree state variables ---
  double current_temp_{25.0};
  double cooling_trigger_{57.0};
  double ammonia_heat_kj_{0.0};
  double vent_threshold_{250.0};

  // BT-specific clients reused from ROS interfaces
  rclcpp::Client<space_station_interfaces::srv::VentHeat>::SharedPtr vent_client_;
  rclcpp::Client<space_station_interfaces::srv::RequestProductWater>::SharedPtr wrs_client_;
};

} // namespace space_station_thermal_control
