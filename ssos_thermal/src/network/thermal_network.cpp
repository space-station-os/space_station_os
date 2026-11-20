#include "ssos_thermal/network/thermal_network.hpp"

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
    node.temperature = entry["initial_temperature_c"] ?
      entry["initial_temperature_c"].as<double>() : reference_temp_c;
    if (entry["max_temperature_c"]) {
      node.max_temperature = entry["max_temperature_c"].as<double>();
    }

    const std::string name = entry["node_name"].as<std::string>();
    if (entry["fixed_temperature_c"]) {
      node.fixed = true;
      node.temperature = entry["fixed_temperature_c"].as<double>();
      net.configured_fixed_c_[name] = node.temperature;
    }
    const std::string parent_link =
      entry["parent_link"] ? entry["parent_link"].as<std::string>() : std::string();
    const double conductance = entry["conductance"].as<double>();

    net.nodes_[name] = node;

    // An empty parent_link marks the root of the tree (e.g. base_link):
    // it has no parent to conduct to, so no link is created for it. Every
    // other node's link.to must itself be a declared node_name for
    // compute_dTdt() to actually exchange heat over it -- see the header
    // comment on config/thermal_nodes.yaml.
    if (!parent_link.empty()) {
      ThermalLinkState link;
      link.from = name;
      link.to = parent_link;
      link.joint_name = name;
      link.conductance = conductance;
      net.links_.push_back(link);
      net.configured_conductance_.push_back(conductance);
    }
  }

  return net;
}

double ThermalNetwork::compute_dTdt(
  const std::string & name, const std::unordered_map<std::string, double> & temps) const
{
  const auto & node = nodes_.at(name);
  if (node.fixed) {
    return 0.0;
  }
  double q_total = node.internal_power + node.external_power;

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

bool ThermalNetwork::set_external_power(const std::string & name, double power_w)
{
  const auto it = nodes_.find(name);
  if (it == nodes_.end()) {
    return false;
  }
  it->second.external_power = power_w;
  return true;
}

void ThermalNetwork::clear_external_power()
{
  for (auto & [name, node] : nodes_) {
    (void)name;
    node.external_power = 0.0;
  }
}

double ThermalNetwork::total_power(const std::string & name) const
{
  const auto it = nodes_.find(name);
  return it == nodes_.end() ? 0.0 : it->second.internal_power + it->second.external_power;
}

double ThermalNetwork::link_heat_flow(const ThermalLinkState & link) const
{
  const auto from = nodes_.find(link.from);
  const auto to = nodes_.find(link.to);
  if (from == nodes_.end() || to == nodes_.end()) {
    return 0.0;
  }
  return link.conductance * (from->second.temperature - to->second.temperature);
}

void ThermalNetwork::set_all_temperatures(double temperature_c)
{
  for (auto & [name, node] : nodes_) {
    (void)name;
    if (!node.fixed) {
      node.temperature = temperature_c;
    }
  }
}

bool ThermalNetwork::set_fixed_temperature(const std::string & name, double temperature_c)
{
  const auto it = nodes_.find(name);
  if (it == nodes_.end() || !it->second.fixed) {
    return false;
  }
  it->second.temperature = temperature_c;
  return true;
}

std::size_t ThermalNetwork::scale_conductance(const std::string & node, double factor)
{
  std::size_t changed = 0;
  for (std::size_t i = 0; i < links_.size(); ++i) {
    if (links_[i].from == node || links_[i].to == node) {
      links_[i].conductance = configured_conductance_[i] * factor;
      ++changed;
    }
  }
  return changed;
}

void ThermalNetwork::clear_overrides()
{
  for (const auto & [name, temperature] : configured_fixed_c_) {
    nodes_.at(name).temperature = temperature;
  }
  for (std::size_t i = 0; i < links_.size(); ++i) {
    links_[i].conductance = configured_conductance_[i];
  }
}

std::string ThermalNetwork::over_limit(double fallback_limit_c, double margin_c) const
{
  for (const auto & [name, node] : nodes_) {
    if (!node.fixed &&
      node.temperature > node.max_temperature.value_or(fallback_limit_c) - margin_c)
    {
      return name;
    }
  }
  return "";
}

double ThermalNetwork::average_temperature() const
{
  double total = 0.0;
  std::size_t count = 0;
  for (const auto & [name, node] : nodes_) {
    (void)name;
    if (!node.fixed) {
      total += node.temperature;
      ++count;
    }
  }
  return count > 0 ? total / static_cast<double>(count) : 0.0;
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
    if (!node.fixed && node.temperature > result.temperature) {
      result.temperature = node.temperature;
      result.name = name;
    }
  }
  return result;
}

}  // namespace network
}  // namespace ssos_thermal
