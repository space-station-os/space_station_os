#ifndef SPACE_STATION_ECLSS__WRS_ACTION_SERVER_HPP_
#define SPACE_STATION_ECLSS__WRS_ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <space_station_eclss/action/water_recovery.hpp>
#include <space_station_eclss/srv/request_product_water.hpp>

namespace space_station_eclss
{

class WRSActionServer : public rclcpp::Node
{
public:
  using WRS = space_station_eclss::action::WaterRecovery;
  using GoalHandleWRS = rclcpp_action::ServerGoalHandle<WRS>;

  explicit WRSActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Action server
  rclcpp_action::Server<WRS>::SharedPtr action_server_;
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const WRS::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleWRS> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleWRS> goal_handle);
  void execute(const std::shared_ptr<GoalHandleWRS> goal_handle);

  // Product Water Tank Service
  rclcpp::Service<space_station_eclss::srv::RequestProductWater>::SharedPtr water_request_server_;
  void handle_product_water_request(
    const std::shared_ptr<space_station_eclss::srv::RequestProductWater::Request> request,
    std::shared_ptr<space_station_eclss::srv::RequestProductWater::Response> response);

  // Gray Water Intake Service (from OGS)
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gray_water_service_;

  // Diagnostic and reserve publishers
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr reserve_pub_;
  rclcpp::TimerBase::SharedPtr reserve_timer_;

  void publish_diagnostics(const std::string & unit, bool failure, const std::string & message);
  void publish_reserve();

  // Internal states
  float product_water_reserve_;
  float waste_collector_current_;

  // Parameters: Failure control and unit properties
  bool enable_failure_;

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
};

}  // namespace space_station_eclss

#endif  // SPACE_STATION_ECLSS__WRS_ACTION_SERVER_HPP_
