#include "thermal_plugin.hh"
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace gz {
namespace sim {
namespace systems {

void ThermalPlugin::Configure(const Entity &entity,
                              const std::shared_ptr<const sdf::Element> & /*sdf*/,
                              EntityComponentManager &ecm,
                              EventManager & /*eventMgr*/)
{
  Model model(entity);
  this->modelName = model.Name(ecm);
  RCLCPP_INFO(rclcpp::get_logger("ThermalPlugin"), "Configuring ThermalPlugin for model: %s", modelName.c_str());

  std::uniform_real_distribution<double> tempDist(290.0, 310.0);
  std::uniform_real_distribution<double> capacityDist(500.0, 1500.0);
  std::uniform_real_distribution<double> powerDist(30.0, 60.0);
  std::uniform_real_distribution<double> conductanceDist(0.05, 2.0);

  ecm.Each<components::Joint, components::Name, components::ParentLinkName, components::ChildLinkName>(
    [&](const Entity & /*jointEnt*/,
        const components::Joint * /*joint*/, const components::Name *name,
        const components::ParentLinkName *parent,
        const components::ChildLinkName *child) -> bool
    {
      std::string jointName = name->Data();
      std::string parentLinkName = parent->Data();
      std::string childLinkName = child->Data();

      if (jointName.empty() || parentLinkName.empty() || childLinkName.empty())
        return true;

      ThermalNode node;
      node.jointName = jointName;
      node.linkName = jointName;
      node.temperature = tempDist(rng);
      node.heatCapacity = capacityDist(rng);
      node.internalPower = powerDist(rng);
      this->thermalNodes[jointName] = node;

      ThermalLink link;
      link.from = childLinkName;
      link.to = parentLinkName;
      link.conductance = conductanceDist(rng);
      this->thermalLinks.push_back(link);

      RCLCPP_INFO(rclcpp::get_logger("ThermalPlugin"),
        "Added node: %s (T=%.2f, C=%.2f, P=%.2f) and link: %s -> %s (G=%.2f)",
        jointName.c_str(), node.temperature, node.heatCapacity, node.internalPower,
        childLinkName.c_str(), parentLinkName.c_str(), link.conductance);

      return true;
    });

  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  ros_node_ = std::make_shared<rclcpp::Node>("thermal_plugin_node");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(ros_node_);

  ros_node_pub_ = ros_node_->create_publisher<space_station_thermal_control::msg::ThermalNodeDataArray>(
    "/thermal/nodes/state", 10);
  ros_link_pub_ = ros_node_->create_publisher<space_station_thermal_control::msg::ThermalLinkFlowsArray>(
    "/thermal/links/flux", 10);
  cooling_service_client_ = ros_node_->create_client<space_station_thermal_control::srv::NodeHeatFlow>("/internal_loop_cooling");

  ros_spin_thread_ = std::thread([this]() {
    executor_->spin();
  });

  RCLCPP_INFO(ros_node_->get_logger(), "ThermalPlugin ROS node initialized.");
}

void ThermalPlugin::PreUpdate(const UpdateInfo &info,
                              EntityComponentManager & /*ecm*/)
{
  if (info.paused)
    return;

  double dt = std::chrono::duration<double>(info.dt).count() * 5.0;

  double total_temp = 0.0;
  double total_power = 0.0;
  int count = 0;

  for (const auto &[name, node] : this->thermalNodes)
  {
    total_temp += node.temperature;
    total_power += node.internalPower;
    ++count;
  }

  avg_temperature_ = total_temp / count;
  avg_internal_power_ = total_power / count;

  RCLCPP_INFO_THROTTLE(this->ros_node_->get_logger(), *this->ros_node_->get_clock(), 10000,
                      "Avg Temp: %.2f K | Avg Power: %.2f W",
                      avg_temperature_, avg_internal_power_);

  // === Safe async cooling service trigger ===
  if (!cooling_active_ && avg_temperature_ > 1300.0)
  {
    if (cooling_service_client_->wait_for_service(1s)) {
      auto req = std::make_shared<space_station_thermal_control::srv::NodeHeatFlow::Request>();
      req->heat_flow = avg_temperature_ - 273.15;

      cooling_service_client_->async_send_request(req,
        [this](rclcpp::Client<space_station_thermal_control::srv::NodeHeatFlow>::SharedFuture future) {
          auto result = future.get();
          if (result->success) {
            RCLCPP_WARN(this->ros_node_->get_logger(), "Cooling triggered: %s", result->message.c_str());
            this->cooling_active_ = true;
          } else {
            RCLCPP_ERROR(this->ros_node_->get_logger(), "Cooling service call failed: %s", result->message.c_str());
          }
        });
    } else {
      RCLCPP_WARN(this->ros_node_->get_logger(), "Cooling service unavailable.");
    }
  }

  // === RK4 Integration ===
  std::unordered_map<std::string, double> k1, k2, k3, k4, T0;
  for (const auto &[name, node] : thermalNodes)
    T0[name] = node.temperature;

  auto compute_dTdt = [&](const std::unordered_map<std::string, double> &temps,
                          const std::string &name) -> double
  {
    const auto &node = thermalNodes[name];
    double q_total = node.internalPower;

    for (const auto &link : thermalLinks) {
      if (link.from == name && temps.count(link.to))
        q_total += link.conductance * (temps.at(link.to) - temps.at(name));
      else if (link.to == name && temps.count(link.from))
        q_total += link.conductance * (temps.at(link.from) - temps.at(name));
    }

    return q_total / node.heatCapacity;
  };

  for (const auto &[name, node] : thermalNodes)
    k1[name] = dt * compute_dTdt(T0, name);

  std::unordered_map<std::string, double> T_k2;
  for (const auto &[name, temp] : T0)
    T_k2[name] = temp + 0.5 * k1[name];

  for (const auto &[name, node] : thermalNodes)
    k2[name] = dt * compute_dTdt(T_k2, name);

  std::unordered_map<std::string, double> T_k3;
  for (const auto &[name, temp] : T0)
    T_k3[name] = temp + 0.5 * k2[name];

  for (const auto &[name, node] : thermalNodes)
    k3[name] = dt * compute_dTdt(T_k3, name);

  std::unordered_map<std::string, double> T_k4;
  for (const auto &[name, temp] : T0)
    T_k4[name] = temp + k3[name];

  for (const auto &[name, node] : thermalNodes)
    k4[name] = dt * compute_dTdt(T_k4, name);

  for (auto &[name, node] : thermalNodes)
    node.temperature += (k1[name] + 2*k2[name] + 2*k3[name] + k4[name]) / 6.0;

  // === Cooling effect ===
  if (cooling_active_) {
    bool all_cooled = true;
    for (auto &[name, node] : thermalNodes) {
      node.temperature -= cooling_rate_ * dt;
      if (node.temperature > 310.0)
        all_cooled = false;
    }

    if (all_cooled) {
      cooling_active_ = false;
      RCLCPP_INFO(ros_node_->get_logger(), "Cooling complete. Returning to normal.");
    }
  }

  // === Publish ROS2 messages ===
  space_station_thermal_control::msg::ThermalNodeDataArray ros_node_msg;
  for (const auto &[name, node] : thermalNodes) {
    space_station_thermal_control::msg::ThermalNodeData msg;
    msg.name = node.linkName;
    msg.temperature = node.temperature;
    msg.heat_capacity = node.heatCapacity;
    msg.internal_power = node.internalPower;
    ros_node_msg.nodes.push_back(msg);
  }
  ros_node_pub_->publish(ros_node_msg);

  space_station_thermal_control::msg::ThermalLinkFlowsArray ros_link_msg;
  for (const auto &link : thermalLinks) {
    space_station_thermal_control::msg::ThermalLinkFlows msg;
    msg.node_a = link.from;
    msg.node_b = link.to;
    msg.conductance = link.conductance;
    msg.heat_flow = 0.0;
    ros_link_msg.links.push_back(msg);
  }
  ros_link_pub_->publish(ros_link_msg);

  RCLCPP_INFO_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 5000,
    "RK4 update complete. Published %zu thermal nodes and %zu links.",
    thermalNodes.size(), thermalLinks.size());
}

GZ_ADD_PLUGIN(ThermalPlugin, System,
              ThermalPlugin::ISystemConfigure,
              ThermalPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(ThermalPlugin, "gz::sim::systems::ThermalPlugin")

}  // namespace systems
}  // namespace sim
}  // namespace gz
