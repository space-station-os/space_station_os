#ifndef SPACE_STATION_ECLSS__WRS_SYSTEMS_HPP_
#define SPACE_STATION_ECLSS__WRS_SYSTEMS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "space_station_eclss/action/water_recovery.hpp"
#include "space_station_eclss/srv/request_product_water.hpp"
#include "space_station_eclss/srv/grey_water.hpp"

#include "std_msgs/msg/float32.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace space_station_eclss
{

class WRSActionServer : public rclcpp::Node
{
public:
  explicit WRSActionServer(const rclcpp::NodeOptions & options);

private:
  using WRS = space_station_eclss::action::WaterRecovery;
  using GoalHandleWRS = rclcpp_action::ServerGoalHandle<WRS>;


  // Action callbacks
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                          std::shared_ptr<const WRS::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleWRS> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleWRS> goal_handle);
  void execute(const std::shared_ptr<GoalHandleWRS> goal_handle);

  // Services
  void handle_product_water_request(
    const std::shared_ptr<space_station_eclss::srv::RequestProductWater::Request> request,
    std::shared_ptr<space_station_eclss::srv::RequestProductWater::Response> response);

  void handle_gray_water_request(
    const std::shared_ptr<space_station_eclss::srv::GreyWater::Request> request,
    std::shared_ptr<space_station_eclss::srv::GreyWater::Response> response);

  // Diagnostics
  void publish_diagnostics(const std::string & unit, bool failure, const std::string & message);
  void publish_reserve();

  // Internal state
  float product_water_reserve_;
  float waste_collector_current_;

  float product_water_capacity_;
  float waste_collector_capacity_;

  float upa_valve_pressure_;
  float ionization_valve_pressure_;
  float filter_valve_pressure_;
  float catalytic_valve_pressure_;

  float upa_max_temperature_;
  float ionization_max_temperature_;
  float filter_max_temperature_;
  float catalytic_max_temperature_;

  bool enable_failure_;

  // Interfaces
  rclcpp_action::Server<WRS>::SharedPtr action_server_;
  rclcpp::Service<space_station_eclss::srv::RequestProductWater>::SharedPtr water_request_server_;
  rclcpp::Service<space_station_eclss::srv::GreyWater>::SharedPtr gray_water_service_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr reserve_pub_;
  rclcpp::TimerBase::SharedPtr reserve_timer_;
};

}  // namespace space_station_eclss

#endif  // SPACE_STATION_ECLSS__WRS_SYSTEMS_HPP_
