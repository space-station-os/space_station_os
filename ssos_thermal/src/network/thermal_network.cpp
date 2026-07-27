#include "ssos_thermal/network/thermal_network.hpp"

#include <cstdlib>
#include <limits>

#include "yaml-cpp/yaml.h"

namespace ssos_thermal
{
namespace network
{

ThermalNetwork ThermalNetwork::load_from_yaml(
  const std::string & filepath, double reference_temp_c)
{
  ThermalNetwork net;
  YAML::Node config = YAML::LoadFile(filepath);

  for (const auto & entry : config) {
    ThermalNodeState node;
    node.heat_capacity = entry["heat_capacity"].as<double>();
    node.internal_power = entry["internal_power"].as<double>();
    node.temperature = reference_temp_c + std::rand() % 10;

    const std::string name = entry["node_name"].as<std::string>();
    const std::string parent_link = entry["parent_link"].as<std::string>();
    const double conductance = entry["conductance"].as<double>();

    net.nodes_[name] = node;

    ThermalLinkState link;
    link.from = name;
    link.to = parent_link;
    link.joint_name = name;
    link.conductance = conductance;
    net.links_.push_back(link);
  }

  return net;
}

double ThermalNetwork::compute_dTdt(
  const std::string & name, const std::unordered_map<std::string, double> & temps) const
{
  const auto & node = nodes_.at(name);
  double q_total = node.internal_power;

  for (const auto & link : links_) {
    if (link.from == name && temps.count(link.to)) {
      q_total += link.conductance * (temps.at(link.to) - temps.at(name));
    } else if (link.to == name && temps.count(link.from)) {
      q_total += link.conductance * (temps.at(link.from) - temps.at(name));
    }
  }

  return q_total / node.heat_capacity;
}

void ThermalNetwork::step(double dt)
{
  if (nodes_.empty()) {
    return;
  }

  std::unordered_map<std::string, double> t0, k1, k2, k3, k4;
  for (const auto & [name, node] : nodes_) {
    t0[name] = node.temperature;
  }
  for (const auto & [name, t] : t0) {
    (void)t;
    k1[name] = dt * compute_dTdt(name, t0);
  }

  std::unordered_map<std::string, double> t_k2;
  for (const auto & [name, t] : t0) {
    t_k2[name] = t + 0.5 * k1[name];
  }
  for (const auto & [name, t] : t0) {
    (void)t;
    k2[name] = dt * compute_dTdt(name, t_k2);
  }

  std::unordered_map<std::string, double> t_k3;
  for (const auto & [name, t] : t0) {
    t_k3[name] = t + 0.5 * k2[name];
  }
  for (const auto & [name, t] : t0) {
    (void)t;
    k3[name] = dt * compute_dTdt(name, t_k3);
  }

  std::unordered_map<std::string, double> t_k4;
  for (const auto & [name, t] : t0) {
    t_k4[name] = t + k3[name];
  }
  for (const auto & [name, t] : t0) {
    (void)t;
    k4[name] = dt * compute_dTdt(name, t_k4);
  }

  for (auto & [name, node] : nodes_) {
    node.temperature += (k1[name] + 2 * k2[name] + 2 * k3[name] + k4[name]) / 6.0;
  }
}

double ThermalNetwork::node_temperature(const std::string & name) const
{
  return nodes_.count(name) ? nodes_.at(name).temperature : 0.0;
}

void ThermalNetwork::set_all_temperatures(double temperature_c)
{
  for (auto & [name, node] : nodes_) {
    (void)name;
    node.temperature = temperature_c;
  }
}

double ThermalNetwork::average_temperature() const
{
  if (nodes_.empty()) {
    return 0.0;
  }
  double total = 0.0;
  for (const auto & [name, node] : nodes_) {
    (void)name;
    total += node.temperature;
  }
  return total / static_cast<double>(nodes_.size());
}

double ThermalNetwork::average_internal_power() const
{
  if (nodes_.empty()) {
    return 0.0;
  }
  double total = 0.0;
  for (const auto & [name, node] : nodes_) {
    (void)name;
    total += node.internal_power;
  }
  return total / static_cast<double>(nodes_.size());
}

ThermalNetwork::Hottest ThermalNetwork::hottest() const
{
  Hottest result{"n/a", -std::numeric_limits<double>::infinity()};
  for (const auto & [name, node] : nodes_) {
    if (node.temperature > result.temperature) {
      result.temperature = node.temperature;
      result.name = name;
    }
  }
  return result;
}

}  // namespace network
}  // namespace ssos_thermal
