#ifndef THERMAL_SOLVER_NODE_HPP_
#define THERMAL_SOLVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <unordered_map>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include "std_msgs/msg/string.hpp"
#include <random>

#include "space_station_thermal_control/msg/thermal_node_data_array.hpp"
#include "space_station_thermal_control/msg/thermal_node_data.hpp"
#include "space_station_thermal_control/msg/thermal_link_flows_array.hpp"
#include "space_station_thermal_control/msg/thermal_link_flows.hpp"
#include "space_station_thermal_control/srv/node_heat_flow.hpp"
#include <chrono>

using namespace std::chrono_literals;
struct ThermalNode
{
  std::string name;
  double temperature;
  double heat_capacity;
  double internal_power;
};

struct ThermalLink
{
  std::string from;
  std::string to;
  double conductance;
};

class ThermalSolverNode : public rclcpp::Node
{
public:
  ThermalSolverNode();
  ~ThermalSolverNode();

private:
  void parseURDF(const std::string &urdf_string);
  void updateSimulation();
  double compute_dTdt(const std::string &name, const std::unordered_map<std::string, double> &temps);
  void coolingCallback();

  std::unordered_map<std::string, ThermalNode> thermal_nodes_;
  std::vector<ThermalLink> thermal_links_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Publisher<space_station_thermal_control::msg::ThermalNodeDataArray>::SharedPtr node_pub_;
  rclcpp::Publisher<space_station_thermal_control::msg::ThermalLinkFlowsArray>::SharedPtr link_pub_;
  rclcpp::Client<space_station_thermal_control::srv::NodeHeatFlow>::SharedPtr cooling_client_;

  bool cooling_active_ = false;
  double cooling_rate_ = 10.0;
  double avg_temperature_ = 0.0;
  double avg_internal_power_ = 0.0;

  std::default_random_engine rng_;
};

#endif  // THERMAL_SOLVER_NODE_HPP_