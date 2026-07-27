#ifndef SSOS_THERMAL__NETWORK__THERMAL_NETWORK_HPP_
#define SSOS_THERMAL__NETWORK__THERMAL_NETWORK_HPP_

#include <string>
#include <unordered_map>
#include <vector>

// Lumped-node thermal network solver. No ROS dependency -- the same object
// can run inside a ROS node (see nodes/thermal_network_node.hpp) or a
// standalone tool.

namespace ssos_thermal
{
namespace network
{

struct ThermalNodeState
{
  double temperature;
  double heat_capacity;
  double internal_power;
};

struct ThermalLinkState
{
  std::string from;
  std::string to;
  std::string joint_name;
  double conductance;
};

class ThermalNetwork
{
public:
  struct Hottest
  {
    std::string name;
    double temperature;
  };

  // Loads node/link definitions from a YAML file shaped like
  // config/thermal_nodes.yaml: one entry per node with node_name,
  // parent_link, heat_capacity, internal_power, conductance. Each node gets
  // a link to its parent_link and an initial temperature randomized around
  // reference_temp_c (matching the legacy solver's startup behavior).
  static ThermalNetwork load_from_yaml(
    const std::string & filepath, double reference_temp_c = 20.0);

  // Advances all node temperatures by dt seconds using classic RK4.
  void step(double dt);

  const std::unordered_map<std::string, ThermalNodeState> & nodes() const {return nodes_;}
  const std::vector<ThermalLinkState> & links() const {return links_;}

  double node_temperature(const std::string & name) const;

  // Used for cooling-loop feedback: snaps every node to the same temperature.
  void set_all_temperatures(double temperature_c);

  double average_temperature() const;
  double average_internal_power() const;
  Hottest hottest() const;

private:
  double compute_dTdt(
    const std::string & name,
    const std::unordered_map<std::string, double> & temps) const;

  std::unordered_map<std::string, ThermalNodeState> nodes_;
  std::vector<ThermalLinkState> links_;
};

}  // namespace network
}  // namespace ssos_thermal

#endif  // SSOS_THERMAL__NETWORK__THERMAL_NETWORK_HPP_
