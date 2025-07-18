#include "space_station_thermal_control/thermal_solver.hpp"
#include <std_msgs/msg/string.hpp>
#include <random>

ThermalSolverNode::ThermalSolverNode()
: Node("thermal_solver_node")
{
  urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/robot_description",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [this](const std_msgs::msg::String::SharedPtr msg)
    {
      parseURDF(msg->data);
    });


  node_pub_ = this->create_publisher<space_station_thermal_control::msg::ThermalNodeDataArray>(
    "/thermal/nodes/state", 10);
  link_pub_ = this->create_publisher<space_station_thermal_control::msg::ThermalLinkFlowsArray>(
    "/thermal/links/flux", 10);
  cooling_client_ = this->create_client<space_station_thermal_control::srv::NodeHeatFlow>(
    "/internal_loop_cooling");

  timer_ = this->create_wall_timer(100ms, std::bind(&ThermalSolverNode::updateSimulation, this));
}

ThermalSolverNode::~ThermalSolverNode() {}

void ThermalSolverNode::parseURDF(const std::string &urdf_string)
{
  urdf::Model model;
  if (!model.initString(urdf_string)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
    return;
  }

  std::uniform_real_distribution<double> temp_dist(290.0, 310.0);
  std::uniform_real_distribution<double> capacity_dist(500.0, 1500.0);
  std::uniform_real_distribution<double> power_dist(30.0, 60.0);
  std::uniform_real_distribution<double> cond_dist(0.05, 2.0);

  thermal_nodes_.clear();
  thermal_links_.clear();

  for (const auto &joint_pair : model.joints_) {
    auto joint = joint_pair.second;
    if (!joint || joint->parent_link_name.empty() || joint->child_link_name.empty())
      continue;

    ThermalNode node;
    node.name = joint->name;
    node.temperature = temp_dist(rng_);
    node.heat_capacity = capacity_dist(rng_);
    node.internal_power = power_dist(rng_);
    thermal_nodes_[joint->name] = node;

    ThermalLink link;
    link.from = joint->child_link_name;
    link.to = joint->parent_link_name;
    link.conductance = cond_dist(rng_);
    thermal_links_.push_back(link);

    RCLCPP_INFO(this->get_logger(), "Added node %s and link %s -> %s",
                joint->name.c_str(), link.from.c_str(), link.to.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Thermal graph initialized with %zu nodes and %zu links.",
              thermal_nodes_.size(), thermal_links_.size());
}

double ThermalSolverNode::compute_dTdt(const std::string &name,
                                       const std::unordered_map<std::string, double> &temps)
{
  const auto &node = thermal_nodes_[name];
  double q_total = node.internal_power;

  for (const auto &link : thermal_links_) {
    if (link.from == name && temps.count(link.to))
      q_total += link.conductance * (temps.at(link.to) - temps.at(name));
    else if (link.to == name && temps.count(link.from))
      q_total += link.conductance * (temps.at(link.from) - temps.at(name));
  }

  return q_total / node.heat_capacity;
}

void ThermalSolverNode::coolingCallback()
{
  if (!cooling_active_ && avg_temperature_ > 400.0) {
    if (cooling_client_->wait_for_service(1s)) {
      auto req = std::make_shared<space_station_thermal_control::srv::NodeHeatFlow::Request>();
      req->heat_flow = avg_temperature_ - 273.15;

      cooling_client_->async_send_request(req,
        [this](rclcpp::Client<space_station_thermal_control::srv::NodeHeatFlow>::SharedFuture future) {
          auto result = future.get();
          if (result->success) {
            RCLCPP_WARN(this->get_logger(), "Cooling triggered: %s", result->message.c_str());
            this->cooling_active_ = true;
          } else {
            RCLCPP_ERROR(this->get_logger(), "Cooling service call failed: %s", result->message.c_str());
          }
        });
    } else {
      RCLCPP_WARN(this->get_logger(), "Cooling service unavailable.");
    }
  }
}

void ThermalSolverNode::updateSimulation()
{
  if (thermal_nodes_.empty())
    return;

  const double dt = 0.5;

  double total_temp = 0.0;
  double total_power = 0.0;
  for (const auto &[name, node] : thermal_nodes_) {
    total_temp += node.temperature;
    total_power += node.internal_power;
  }

  avg_temperature_ = total_temp / thermal_nodes_.size();
  avg_internal_power_ = total_power / thermal_nodes_.size();

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, 
                     "Avg temperature of node = %.2f", avg_temperature_);


  coolingCallback();

  std::unordered_map<std::string, double> T0, k1, k2, k3, k4;
  for (const auto &[name, node] : thermal_nodes_)
    T0[name] = node.temperature;

  for (const auto &[name, node] : thermal_nodes_)
    k1[name] = dt * compute_dTdt(name, T0);

  std::unordered_map<std::string, double> T_k2;
  for (const auto &[name, t] : T0)
    T_k2[name] = t + 0.5 * k1[name];

  for (const auto &[name, node] : thermal_nodes_)
    k2[name] = dt * compute_dTdt(name, T_k2);

  std::unordered_map<std::string, double> T_k3;
  for (const auto &[name, t] : T0)
    T_k3[name] = t + 0.5 * k2[name];

  for (const auto &[name, node] : thermal_nodes_)
    k3[name] = dt * compute_dTdt(name, T_k3);

  std::unordered_map<std::string, double> T_k4;
  for (const auto &[name, t] : T0)
    T_k4[name] = t + k3[name];

  for (const auto &[name, node] : thermal_nodes_)
    k4[name] = dt * compute_dTdt(name, T_k4);

  for (auto &[name, node] : thermal_nodes_)
    node.temperature += (k1[name] + 2 * k2[name] + 2 * k3[name] + k4[name]) / 6.0;

  if (cooling_active_) {
    bool all_cooled = true;
    for (auto &[name, node] : thermal_nodes_) {
      node.temperature -= cooling_rate_ * dt;
      if (node.temperature > 310.0)
        all_cooled = false;
    }
    if (all_cooled) {
      cooling_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Cooling complete. Returning to normal.");
    }
  }

  // Publish ROS messages
  space_station_thermal_control::msg::ThermalNodeDataArray node_msg;
  for (const auto &[name, node] : thermal_nodes_) {
    space_station_thermal_control::msg::ThermalNodeData data;
    data.name = name;
    data.temperature = node.temperature;
    data.heat_capacity = node.heat_capacity;
    data.internal_power = node.internal_power;
    node_msg.nodes.push_back(data);
  }
  node_pub_->publish(node_msg);

  space_station_thermal_control::msg::ThermalLinkFlowsArray link_msg;
  for (const auto &link : thermal_links_) {
    space_station_thermal_control::msg::ThermalLinkFlows l;
    l.node_a = link.from;
    l.node_b = link.to;
    l.conductance = link.conductance;
    l.heat_flow = 0.0;  // optional
    link_msg.links.push_back(l);
  }
  link_pub_->publish(link_msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalSolverNode>());
  rclcpp::shutdown();
  return 0;
}