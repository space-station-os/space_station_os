#ifndef SSOS_GNC__NODES__GNC_PARAMETER_BRIDGE_HPP_
#define SSOS_GNC__NODES__GNC_PARAMETER_BRIDGE_HPP_

#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "ssos_gnc/flight/control/control_parameters.hpp"
#include "ssos_gnc/plant/disturbance_torques.hpp"

namespace ssos_gnc
{

namespace nodes
{

class GncParameterBridge
{
public:
  explicit GncParameterBridge(rclcpp_lifecycle::LifecycleNode * node);

  void declare_flight_parameters(const flight::GncParameters & defaults);

  void declare_plant_parameters(const plant::DisturbanceParams & defaults);

  flight::GncParameters read_flight_parameters() const;

  plant::DisturbanceParams read_disturbance_parameters() const;

  Eigen::Matrix3d read_inertia_tensor() const;

  rcl_interfaces::msg::SetParametersResult validate(
    const std::vector<rclcpp::Parameter> & params) const;

  static bool is_static_parameter(const std::string & name);

private:
  double get_double(const std::string & name, double fallback) const;
  bool get_bool(const std::string & name, bool fallback) const;

  rclcpp_lifecycle::LifecycleNode * node_;
};
}  // namespace nodes
}  // namespace ssos_gnc

#endif  // SSOS_GNC__NODES__GNC_PARAMETER_BRIDGE_HPP_
