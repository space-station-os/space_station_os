#include "space_station_thermal_control/thermal_solver.hpp"

using namespace std::chrono_literals;

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
  this->declare_parameter("cooling_trigger_threshold", 57.0);  // Celsius
  this->declare_parameter("max_temp_threshold", 147.0);        // Celsius
  this->declare_parameter("cooling_rate", 0.05);
  this->declare_parameter("thermal_update_dt", 0.3);
  this->declare_parameter<std::string>("thermal_config_file", "config/thermal_nodes.yaml");

  enable_failure_ = this->get_parameter("enable_failure").as_bool();
  enable_cooling_ = this->get_parameter("enable_cooling").as_bool();
  cooling_trigger_threshold_ = this->get_parameter("cooling_trigger_threshold").as_double();
  max_temp_threshold_ = this->get_parameter("max_temp_threshold").as_double();
  cooling_rate_ = this->get_parameter("cooling_rate").as_double();
  thermal_update_dt_ = this->get_parameter("thermal_update_dt").as_double();
  cooling_active_ = false;

  param_callback_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &params) {
      for (const auto &param : params) {
        if (param.get_name() == "enable_failure")
          enable_failure_ = param.as_bool();
        else if (param.get_name() == "enable_cooling")
          enable_cooling_ = param.as_bool();
        else if (param.get_name() == "cooling_trigger_threshold")
          cooling_trigger_threshold_ = param.as_double();
        else if (param.get_name() == "max_temp_threshold")
          max_temp_threshold_ = param.as_double();
        else if (param.get_name() == "cooling_rate")
          cooling_rate_ = param.as_double();
        else if (param.get_name() == "thermal_update_dt") {
          thermal_update_dt_ = param.as_double();
          timer_->cancel();
          timer_ = this->create_wall_timer(
            std::chrono::duration<double>(thermal_update_dt_),
            std::bind(&ThermalSolverNode::updateSimulation, this));
        }
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

  std::string relative_path = this->get_parameter("thermal_config_file").as_string();
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("space_station_thermal_control");
  std::string config_path = package_share_dir + "/" + relative_path;

  parseYAMLConfig(config_path);

  timer_ = this->create_wall_timer(std::chrono::duration<double>(thermal_update_dt_), std::bind(&ThermalSolverNode::updateSimulation, this));

}


ThermalSolverNode::~ThermalSolverNode() {}

void ThermalSolverNode::parseYAMLConfig(const std::string &filepath)
{
  RCLCPP_INFO(this->get_logger(), "Parsing thermal config: %s", filepath.c_str());

  YAML::Node config = YAML::LoadFile(filepath);

  for (const auto &entry : config) {
    ThermalNode node;
    node.heat_capacity = entry["heat_capacity"].as<double>();
    node.internal_power = entry["internal_power"].as<double>();
    node.temperature = REFERENCE_TEMP_CELCIUS + std::rand() % 10;  // Celsius
    std::string name = entry["node_name"].as<std::string>();
    std::string parent_link = entry["parent_link"].as<std::string>();
    double conductance = entry["conductance"].as<double>();

    thermal_nodes_[name] = node;
    initial_temperatures_[name] = node.temperature;

    ThermalLink link;
    link.from = name;
    link.to = parent_link;
    link.joint_name = name;
    link.conductance = conductance;

    thermal_links_.push_back(link);
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %zu thermal nodes and %zu thermal links.",
              thermal_nodes_.size(), thermal_links_.size());
}

double ThermalSolverNode::compute_dTdt(
  const std::string &name,
  const std::unordered_map<std::string, double> &temps)
{
  const auto &node = thermal_nodes_.at(name);
  double q_total = node.internal_power;

  for (const auto &link : thermal_links_) {
    if (link.from == name && temps.count(link.to)) {
      q_total += link.conductance * (temps.at(link.to) - temps.at(name));
    } else if (link.to == name && temps.count(link.from)) {
      q_total += link.conductance * (temps.at(link.from) - temps.at(name));
    }
  }

  return q_total / node.heat_capacity;
}


void ThermalSolverNode::coolingCallback()
{
  
  if (!cooling_active_ && enable_cooling_ && avg_temperature_ > cooling_trigger_threshold_) {
    if (cooling_client_->wait_for_service(1s)) {
      auto req = std::make_shared<space_station_thermal_control::srv::NodeHeatFlow::Request>();
      req->heat_flow = avg_temperature_ - REFERENCE_TEMP_CELCIUS;

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
  if (thermal_nodes_.empty()) return;
  double total_temp = 0.0;
  double total_power = 0.0;
  for (const auto &[name, node] : thermal_nodes_) {
    total_temp += node.temperature;
    total_power += node.internal_power;
  }

  avg_temperature_ = total_temp / thermal_nodes_.size();
  avg_internal_power_ = total_power / thermal_nodes_.size();

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                       "Avg temperature = %.2f K", avg_temperature_);

  coolingCallback();

  std::unordered_map<std::string, double> T0, k1, k2, k3, k4;
  for (const auto &[name, node] : thermal_nodes_)
    T0[name] = node.temperature;

  for (const auto &[name, node] : thermal_nodes_)
    k1[name] = thermal_update_dt_ * compute_dTdt(name, T0);

  std::unordered_map<std::string, double> T_k2;
  for (const auto &[name, t] : T0)
    T_k2[name] = t + 0.5 * k1[name];

  for (const auto &[name, node] : thermal_nodes_)
    k2[name] = thermal_update_dt_ * compute_dTdt(name, T_k2);

  std::unordered_map<std::string, double> T_k3;
  for (const auto &[name, t] : T0)
    T_k3[name] = t + 0.5 * k2[name];

  for (const auto &[name, node] : thermal_nodes_)
    k3[name] = thermal_update_dt_ * compute_dTdt(name, T_k3);

  std::unordered_map<std::string, double> T_k4;
  for (const auto &[name, t] : T0)
    T_k4[name] = t + k3[name];

  for (const auto &[name, node] : thermal_nodes_)
    k4[name] = thermal_update_dt_ * compute_dTdt(name, T_k4);

  double hottest_temp = -std::numeric_limits<double>::infinity();
  std::string hottest_name = "n/a";

  for (auto &[name, node] : thermal_nodes_) {
    node.temperature += (k1[name] + 2 * k2[name] + 2 * k3[name] + k4[name]) / 6.0;
    if (node.temperature > hottest_temp) {
      hottest_temp = node.temperature;
      hottest_name = name;
    }
  }


  if (cooling_active_) {
    for (auto &[name, node] : thermal_nodes_) {
      double target_temp = initial_temperatures_[name];
      double diff = node.temperature - target_temp;
      if (std::abs(diff) > 0.5) {
        node.temperature -= std::copysign(cooling_rate_ * thermal_update_dt_, diff);
      } else {
        node.temperature = target_temp;
      }

      if (enable_failure_ && node.temperature > max_temp_threshold_) {
        diagnostic_msgs::msg::DiagnosticStatus diag;
        diag.name = "THERMAL_NODE_OVERHEAT";
        diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag.message = "Overheating after cooling";
        diag.hardware_id = name;

        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "temperature_C";
        kv.value = std::to_string(node.temperature);
        diag.values.push_back(kv);

        diag_pub_->publish(diag);
      }
    }

    
    bool all_below = true;
    for (const auto &[name, node] : thermal_nodes_) {
      if (node.temperature > initial_temperatures_[name] + 0.5) {
        all_below = false;
        break;
      }
    }

    if (all_below) {
      cooling_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Cooling complete, system stabilized.");
    }
  }



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
  publishThermalNetworkDiag(node_msg.nodes);

  space_station_thermal_control::msg::ThermalLinkFlowsArray link_msg;
  for (const auto &link : thermal_links_) {
    space_station_thermal_control::msg::ThermalLinkFlows l;
    l.node_a = link.from;
    l.node_b = link.to;
    l.conductance = link.conductance;

    double T_a = thermal_nodes_.count(link.joint_name) ? thermal_nodes_.at(link.joint_name).temperature : 0.0;
    double T_b = 20.0;  // Reference temp in Celsius

    l.heat_flow = link.conductance * (T_a - T_b);
    link_msg.links.push_back(l);
  }
  link_pub_->publish(link_msg);
}

void ThermalSolverNode::publishThermalNetworkDiag(
    const std::vector<space_station_thermal_control::msg::ThermalNodeData> &nodes)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using diagnostic_msgs::msg::KeyValue;

  DiagnosticStatus st;
  st.name = "ThermalNetwork";

  std::string hottest_name = "n/a";
  double hottest_temp = -std::numeric_limits<double>::infinity();

  for (const auto &n : nodes) {
    if (n.temperature > hottest_temp) {
      hottest_temp = n.temperature;
      hottest_name = n.name;
    }
  }

  if (std::isfinite(hottest_temp) && hottest_temp > max_temp_threshold_) {
    st.level = DiagnosticStatus::WARN;
    st.message = "Node temperature exceeds max_temp_threshold";
  } else {
    st.level = DiagnosticStatus::OK;
    st.message = "All node temperatures within threshold";
  }

  KeyValue kv1, kv2;
  kv1.key = "hottest_node";
  kv1.value = hottest_name;

  kv2.key = "node_temperature_C";
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << hottest_temp;
    kv2.value = oss.str();
  }

  st.hardware_id = hottest_name;
  st.values = {kv1, kv2};

  diag_pub_->publish(st);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalSolverNode>());
  rclcpp::shutdown();
  return 0;
}
