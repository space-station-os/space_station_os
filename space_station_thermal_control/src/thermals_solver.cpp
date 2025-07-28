#include "space_station_thermal_control/thermal_solver.hpp"



ThermalSolverNode::ThermalSolverNode()
: Node("thermal_solver_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing Thermal Solver Node...");
  node_pub_ = this->create_publisher<space_station_thermal_control::msg::ThermalNodeDataArray>(
    "/thermal/nodes/state", 10);
  link_pub_ = this->create_publisher<space_station_thermal_control::msg::ThermalLinkFlowsArray>(
    "/thermal/links/flux", 10);
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "/thermals/diagnostics", 10);
  cooling_client_ = this->create_client<space_station_thermal_control::srv::NodeHeatFlow>(
    "/internal_loop_cooling");

  this->declare_parameter("enable_failure", false);
  this->declare_parameter("enable_cooling", true);
  enable_failure_ = this->get_parameter("enable_failure").as_bool();
  enable_cooling_ = this->get_parameter("enable_cooling").as_bool();

  param_callback_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        for (const auto &param : params) {
          if (param.get_name() == "enable_failure")
            enable_failure_ = param.as_bool();
          else if (param.get_name() == "enable_cooling")
            enable_cooling_ = param.as_bool();
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });
  this->declare_parameter<std::string>("thermal_config_file", "./config/thermal_nodes.yaml");
  std::string relative_path = this->get_parameter("thermal_config_file").as_string();

  std::string package_share_dir = ament_index_cpp::get_package_share_directory("space_station_thermal_control");
  std::string config_path = package_share_dir + "/" + relative_path;

  parseYAMLConfig(config_path);

  timer_ = this->create_wall_timer(100ms, std::bind(&ThermalSolverNode::updateSimulation, this));
}

ThermalSolverNode::~ThermalSolverNode() {}


void ThermalSolverNode::parseYAMLConfig(const std::string &yaml_path)
{
  YAML::Node config = YAML::LoadFile(yaml_path);
  if (!config || !config.IsSequence()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid YAML configuration.");
    return;
  }

  thermal_nodes_.clear();
  thermal_links_.clear();
  initial_temperatures_.clear();

  for (const auto &entry : config) {
    std::string node_name = entry["node_name"].as<std::string>();
    std::string parent_link = entry["parent_link"].as<std::string>();
    double heat_capacity = entry["heat_capacity"].as<double>();
    double internal_power = entry["internal_power"].as<double>();
    double conductance = entry["conductance"].as<double>();

    if (!thermal_nodes_.count(parent_link) && parent_link != "base_link") {
      RCLCPP_WARN(this->get_logger(), "Parent link '%s' not defined as a node.", 
      parent_link.c_str());
    }

    ThermalNode node;
    node.name = node_name;
    node.temperature = 293.15 + static_cast<double>(rand() % 10); // Init near room temp
    node.heat_capacity = heat_capacity;
    node.internal_power = internal_power;
    thermal_nodes_[node_name] = node;
    initial_temperatures_[node_name] = node.temperature;

    ThermalLink link;
    link.from = node_name;
    link.to = parent_link;
    link.conductance = conductance;
    link.joint_name = node_name;
    thermal_links_.push_back(link);

    RCLCPP_INFO(this->get_logger(), "Added node %s and link %s -> %s",
                node_name.c_str(), link.from.c_str(), link.to.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "YAML thermal config loaded: %zu nodes, %zu links.",
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
  if (enable_failure_ && avg_temperature_ > 420.0) {
    diagnostic_msgs::msg::DiagnosticStatus diag;
    diag.name = "THERMAL_SOLVER_OVERHEAT";
    diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag.message = "Thermal system overheating";
    diag_pub_->publish(diag);
    RCLCPP_ERROR(this->get_logger(), "Thermal system overheating, triggering cooling.");
  }

  if (!cooling_active_ && enable_cooling_ && avg_temperature_ > 330.0) {
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
      double target_temp = initial_temperatures_[name];
      double diff = node.temperature - target_temp;

      if (std::abs(diff) < 0.5) {
        node.temperature = target_temp;
      } else {
        node.temperature -= std::copysign(cooling_rate_ * dt, diff);
        all_cooled = false;
      }
    }

    if (all_cooled) {
      RCLCPP_INFO(this->get_logger(), "All nodes restored to initial temperatures.");
      cooling_active_ = false;
    }
  }


  // Publish node temperatures
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

  // Publish heat flux across links
  space_station_thermal_control::msg::ThermalLinkFlowsArray link_msg;
      for (const auto &link : thermal_links_) {
      space_station_thermal_control::msg::ThermalLinkFlows l;
      l.node_a = link.from;
      l.node_b = link.to;
      l.conductance = link.conductance;

      // Retrieve temperature of the joint itself (as node_a)
      double T_a = 0.0;
      if (auto it = thermal_nodes_.find(link.joint_name); it != thermal_nodes_.end()) {
        T_a = it->second.temperature;
      }

      // Simulate base_link (or surrounding environment) as fixed temp
      double T_b = 293.15;  

      l.heat_flow = link.conductance * (T_a - T_b);
      link_msg.links.push_back(l);
    }
   link_pub_->publish(link_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalSolverNode>());
  rclcpp::shutdown();
  return 0;
}
