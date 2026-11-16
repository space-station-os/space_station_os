#ifndef SSOS_EPS__NODES__EPS_PARAMETER_BRIDGE_HPP_
#define SSOS_EPS__NODES__EPS_PARAMETER_BRIDGE_HPP_

#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "ssos_eps/system/eps_parameters.hpp"

// The single mapping between ROS 2 parameters and the EPS physics structs.
// Parameter names mirror the struct layout (solar.array.*, solar.ssu.*,
// battery.*, bcdu.*, eps.*). Static parameters change the channel/ORU
// topology or the battery's initial state and are read in on_configure;
// everything else is live-tunable and validated before acceptance.

namespace ssos_eps
{
namespace nodes
{

class EpsParameterBridge
{
public:
  explicit EpsParameterBridge(rclcpp_lifecycle::LifecycleNode * node);

  /// Declare every EPS physics parameter using the supplied defaults.
  void declare_parameters(const system::EpsParameters & defaults);

  /// Build the physics parameter set from the node's current parameters.
  system::EpsParameters read_parameters() const;

  /// Validate a proposed set of parameter changes against physical ranges and
  /// against the current values of related parameters.
  rcl_interfaces::msg::SetParametersResult validate(
    const std::vector<rclcpp::Parameter> & params) const;

  /// True if the parameter belongs to the EPS physics set.
  static bool is_physics_parameter(const std::string & name);

  /// True if a parameter only takes effect after a reconfigure.
  static bool is_static_parameter(const std::string & name);

private:
  double proposed_double(const std::string & name,
                         const std::vector<rclcpp::Parameter> & params) const;
  std::vector<double> proposed_array(const std::string & name,
                                     const std::vector<rclcpp::Parameter> & params) const;

  rclcpp_lifecycle::LifecycleNode * node_;
};

}  // namespace nodes
}  // namespace ssos_eps

#endif  // SSOS_EPS__NODES__EPS_PARAMETER_BRIDGE_HPP_
