#ifndef THERMAL_SOLVER_NODE_HPP_
#define THERMAL_SOLVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <random>

#include "std_msgs/msg/string.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "space_station_thermal_control/msg/thermal_node_data_array.hpp"
#include "space_station_thermal_control/msg/thermal_node_data.hpp"
#include "space_station_thermal_control/msg/thermal_link_flows_array.hpp"
#include "space_station_thermal_control/msg/thermal_link_flows.hpp"
#include "space_station_thermal_control/srv/node_heat_flow.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <space_station_thermal_control/action/coolant.hpp>

#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

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
  std::string joint_name;
};

class ThermalSolverNode : public rclcpp::Node
{
public:
  ThermalSolverNode();
  ~ThermalSolverNode();

private:
  using GoalHandleCoolant = rclcpp_action::ClientGoalHandle<space_station_thermal_control::action::Coolant>;

  void parseYAMLConfig(const std::string &yaml_path);
  void updateSimulation();
  double compute_dTdt(const std::string &name, const std::unordered_map<std::string, double> &temps);
  void coolingCallback();
  void publishThermalNetworkDiag(const std::vector<space_station_thermal_control::msg::ThermalNodeData> &nodes);
  // ROS publishers and clients
  rclcpp::Publisher<space_station_thermal_control::msg::ThermalNodeDataArray>::SharedPtr node_pub_;
  rclcpp::Publisher<space_station_thermal_control::msg::ThermalLinkFlowsArray>::SharedPtr link_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp_action::Client<space_station_thermal_control::action::Coolant>::SharedPtr cooling_client_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  // Thermal configuration and state
  std::unordered_map<std::string, ThermalNode> thermal_nodes_;
  std::vector<ThermalLink> thermal_links_;
  std::unordered_map<std::string, double> initial_temperatures_;

  // Dynamic parameters
  bool enable_failure_ = false;
  bool enable_cooling_ = true;
  bool cooling_active_ = false;
  double  REFERENCE_TEMP_CELCIUS = 20.0;
  double cooling_rate_ = 0.05;
  double cooling_trigger_threshold_ = 330.0;
  double max_temp_threshold_ = 420.0;
  double thermal_update_dt_ = 0.5;

  // Simulation metrics
  double avg_temperature_ = 0.0;
  double avg_internal_power_ = 0.0;

  std::string config_path_;
  std::default_random_engine rng_;
};

#endif  // THERMAL_SOLVER_NODE_HPP_
