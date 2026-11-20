#ifndef SSOS_THERMAL__NETWORK__THERMAL_NETWORK_HPP_
#define SSOS_THERMAL__NETWORK__THERMAL_NETWORK_HPP_

#include <optional>
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
  double external_power{0.0};   // heat added by other subsystems [W]
  bool fixed{false};            // boundary node held at its temperature
  std::optional<double> max_temperature;  // per-node limit [degC]
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
  // parent_link, heat_capacity, internal_power, conductance. Each node with
  // a non-empty parent_link gets a conductive link to it; an empty (or
  // omitted) parent_link marks a root node (e.g. base_link) with no link of
  // its own. A node starts at initial_temperature_c if given, otherwise at
  // reference_temp_c. max_temperature_c is an optional per-node limit used by
  // over_limit().
  //
  // A link's "to" must itself be a declared node_name for compute_dTdt() to
  // exchange heat over it -- a link pointing at an undeclared name is inert.
  //
  // A node with fixed_temperature_c is a boundary (e.g. the heat sink of a
  // coolant loop): it stays at that temperature and is left out of the
  // average, the hottest-node search and set_all_temperatures().
  static ThermalNetwork load_from_yaml(
    const std::string & filepath, double reference_temp_c = 20.0);

  // Advances all node temperatures by dt seconds using classic RK4.
  void step(double dt);

  const std::unordered_map<std::string, ThermalNodeState> & nodes() const {return nodes_;}
  const std::vector<ThermalLinkState> & links() const {return links_;}

  double node_temperature(const std::string & name) const;

  // Heat other subsystems dissipate into a node, on top of its configured
  // internal_power [W]. Returns false for an unknown node.
  bool set_external_power(const std::string & name, double power_w);
  void clear_external_power();

  // internal_power + external_power of a node [W].
  double total_power(const std::string & name) const;

  // Heat flow over a link [W], positive when heat moves from link.from to
  // link.to: conductance * (T_from - T_to). Same relation step() integrates,
  // so telemetry and physics agree. 0 for an inert link (an end that isn't a
  // declared node), matching compute_dTdt(), which skips such links.
  double link_heat_flow(const ThermalLinkState & link) const;

  // Used for cooling-loop feedback: snaps every node to the same temperature.
  void set_all_temperatures(double temperature_c);

  // Overrides for fault drills, undone by clear_overrides(). A boundary
  // node can be held at another temperature (false if the node is not
  // fixed), and every link touching a node can run at factor times its
  // configured conductance (returns the number of links changed).
  bool set_fixed_temperature(const std::string & name, double temperature_c);
  std::size_t scale_conductance(const std::string & node, double factor);
  void clear_overrides();

  // First non-boundary node above its limit minus margin_c, or "" if none.
  // Nodes without max_temperature_c use fallback_limit_c.
  std::string over_limit(double fallback_limit_c, double margin_c = 0.0) const;

  double average_temperature() const;
  double average_internal_power() const;
  Hottest hottest() const;

private:
  double compute_dTdt(
    const std::string & name,
    const std::unordered_map<std::string, double> & temps) const;

  std::unordered_map<std::string, ThermalNodeState> nodes_;
  std::vector<ThermalLinkState> links_;
  // Configured values the drill overrides are measured from.
  std::unordered_map<std::string, double> configured_fixed_c_;
  std::vector<double> configured_conductance_;
};

}  // namespace network
}  // namespace ssos_thermal

#endif  // SSOS_THERMAL__NETWORK__THERMAL_NETWORK_HPP_
