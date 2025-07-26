#ifndef SPACE_STATION_ECLSS__WRS_SYSTEMS_HPP_
#define SPACE_STATION_ECLSS__WRS_SYSTEMS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_msgs/msg/float64.hpp>

#include "space_station_eclss/action/water_recovery.hpp"
#include "space_station_eclss/action/oxygen_generation.hpp"
#include "space_station_eclss/srv/request_product_water.hpp"
#include "space_station_eclss/srv/grey_water.hpp"
#include <std_msgs/msg/bool.hpp>

#include <memory>
#include <string>

namespace space_station_eclss
{

class WRSActionServer : public rclcpp::Node
{
public:
  explicit WRSActionServer(const rclcpp::NodeOptions & options);

private:
  // Action types
  using WRS = space_station_eclss::action::WaterRecovery;
  using GoalHandleWRS = rclcpp_action::ServerGoalHandle<WRS>;

  using OGS = space_station_eclss::action::OxygenGeneration;
  using GoalHandleOGS = rclcpp_action::ClientGoalHandle<OGS>;

  // Action server
  rclcpp_action::Server<WRS>::SharedPtr action_server_;

  // Action client
  rclcpp_action::Client<OGS>::SharedPtr ogs_client_;

  // Service servers
  rclcpp::Service<space_station_eclss::srv::RequestProductWater>::SharedPtr water_request_server_;
  rclcpp::Service<space_station_eclss::srv::GreyWater>::SharedPtr gray_water_service_;

  // Publishers
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr reserve_pub_;
  rclcpp::TimerBase::SharedPtr reserve_timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr disable_failure_;
  // Internal state
  float product_water_reserve_;
  float waste_collector_current_;
  float min_product_water_capacity_;
  // System parameters
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

  // Action server handlers
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const WRS::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleWRS> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleWRS> goal_handle);
  void execute(const std::shared_ptr<GoalHandleWRS> goal_handle);

  // Helper functions
  void fail_goal(const std::shared_ptr<GoalHandleWRS> & goal_handle,
                 std::shared_ptr<WRS::Result> & result,
                 int cycles,
                 float total,
                 const std::string & reason);

  void send_water_to_ogs(float volume, float iodine_ppm = 0.0f);

  void handle_product_water_request(
    const std::shared_ptr<space_station_eclss::srv::RequestProductWater::Request> request,
    std::shared_ptr<space_station_eclss::srv::RequestProductWater::Response> response);

  void handle_gray_water_request(
    const std::shared_ptr<space_station_eclss::srv::GreyWater::Request> request,
    std::shared_ptr<space_station_eclss::srv::GreyWater::Response> response);

  void publish_diagnostics(const std::string & unit, bool failure, const std::string & message);
  void publish_reserve();
};

}  // namespace space_station_eclss

#endif  // SPACE_STATION_ECLSS__WRS_SYSTEMS_HPP_
