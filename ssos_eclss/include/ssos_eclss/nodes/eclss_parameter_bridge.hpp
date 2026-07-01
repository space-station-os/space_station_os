#ifndef SSOS_ECLSS__NODES__ECLSS_PARAMETER_BRIDGE_HPP_
#define SSOS_ECLSS__NODES__ECLSS_PARAMETER_BRIDGE_HPP_

#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "ssos_eclss/ars/ars_parameters.hpp"

// The single mapping between ROS 2 parameters and the ECLSS physics structs.
// Parameters flow IN from ROS; the physics library never hardcodes them
// (factory functions supply defaults only). Static parameters (geometry,
// n_cells) are read once in on_configure; dynamic parameters (heater, cycle,
// flow, isotherm what-if) are live-tunable and validated before acceptance.

namespace ssos_eclss
{
namespace nodes
{

class EclssParameterBridge
{
public:
  explicit EclssParameterBridge(rclcpp_lifecycle::LifecycleNode * node);

  /// Declare all ARS parameters using the supplied defaults.
  void declare_ars_parameters(const ars::ArsParameters & defaults);

  /// Read the full ARS parameter set from the node's current parameters.
  ars::ArsParameters read_ars_parameters() const;

  /// Validate a proposed set of parameter changes. Returns successful=false
  /// with a clear reason if any value is out of physical range.
  rcl_interfaces::msg::SetParametersResult validate(
    const std::vector<rclcpp::Parameter> & params) const;

  /// True if a parameter name is static (requires reconfigure to take effect).
  static bool is_static_parameter(const std::string & name);

private:
  rclcpp_lifecycle::LifecycleNode * node_;
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__ECLSS_PARAMETER_BRIDGE_HPP_
