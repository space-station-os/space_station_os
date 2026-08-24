#include "ssos_gnc/nodes/gnc_parameter_bridge.hpp"

#include <algorithm>
#include <cmath>

namespace ssos_gnc
{

namespace nodes
{

namespace
{
const char * kStaticParameters[] = {
  "inertia.ixx", "inertia.iyy", "inertia.izz",
  "inertia.ixy", "inertia.ixz", "inertia.iyz",
  "thrusters.urdf_param", "thrusters.table_path", "thrusters.table_name",
  "step_rate_hz",
};
}  // namespace nodes

GncParameterBridge::GncParameterBridge(rclcpp_lifecycle::LifecycleNode * node)
: node_(node)
{
}  // namespace ssos_gnc

bool GncParameterBridge::is_static_parameter(const std::string & name)
{
  for (const char * s : kStaticParameters) {
    if (name == s) {return true;}
  }
  return false;
}

double GncParameterBridge::get_double(const std::string & name, double fallback) const
{
  double v = fallback;
  node_->get_parameter_or(name, v, fallback);
  return v;
}

bool GncParameterBridge::get_bool(const std::string & name, bool fallback) const
{
  bool v = fallback;
  node_->get_parameter_or(name, v, fallback);
  return v;
}

void GncParameterBridge::declare_flight_parameters(const flight::GncParameters & d)
{
  node_->declare_parameter("control.cmg.kp", d.cmg.kp);
  node_->declare_parameter("control.cmg.kd", d.cmg.kd);
  node_->declare_parameter("control.thruster.kp", d.thruster.kp);
  node_->declare_parameter("control.thruster.kd", d.thruster.kd);

  node_->declare_parameter("control.filter.angle_alpha", d.filter.angle_alpha);
  node_->declare_parameter("control.filter.rate_alpha", d.filter.rate_alpha);

  node_->declare_parameter("control.dead_zone.enabled", d.dead_zone.enabled);
  node_->declare_parameter("control.dead_zone.angle_on_deg", d.dead_zone.angle_on_deg);
  node_->declare_parameter("control.dead_zone.angle_off_deg", d.dead_zone.angle_off_deg);
  node_->declare_parameter("control.dead_zone.rate_on_dps", d.dead_zone.rate_on_dps);
  node_->declare_parameter("control.dead_zone.rate_off_dps", d.dead_zone.rate_off_dps);

  node_->declare_parameter("control.unload.gain", d.unload.gain);
  node_->declare_parameter("control.unload.completion_threshold", d.unload.completion_threshold);

  node_->declare_parameter("modes.min_dwell_s", d.mode.min_dwell_s);
  node_->declare_parameter("actuators.gimbal_rate_limit", d.gimbal_rate_limit);
}

void GncParameterBridge::declare_plant_parameters(const plant::DisturbanceParams & d)
{
  const plant::Inertia inertia = plant::default_inertia();
  node_->declare_parameter("inertia.ixx", inertia.tensor()(0, 0));
  node_->declare_parameter("inertia.iyy", inertia.tensor()(1, 1));
  node_->declare_parameter("inertia.izz", inertia.tensor()(2, 2));
  node_->declare_parameter("inertia.ixy", inertia.tensor()(0, 1));
  node_->declare_parameter("inertia.ixz", inertia.tensor()(0, 2));
  node_->declare_parameter("inertia.iyz", inertia.tensor()(1, 2));

  node_->declare_parameter("disturbance.enable_gravity_gradient", d.enable_gravity_gradient);
  node_->declare_parameter("disturbance.enable_aero_drag", d.enable_aero_drag);
  node_->declare_parameter("disturbance.enable_srp", d.enable_srp);
  node_->declare_parameter("disturbance.drag_coefficient", d.drag_coefficient);
  node_->declare_parameter("disturbance.frontal_area_m2", d.frontal_area_m2);
  node_->declare_parameter("disturbance.cop_offset_m", d.center_of_pressure_offset_m);
  node_->declare_parameter("disturbance.srp_area_m2", d.srp_area_m2);
  node_->declare_parameter("disturbance.srp_reflectivity", d.srp_reflectivity);
  node_->declare_parameter("disturbance.srp_center_offset_m", d.srp_center_offset_m);
}

flight::GncParameters GncParameterBridge::read_flight_parameters() const
{
  flight::GncParameters p = flight::default_gnc_parameters();

  p.cmg.kp = get_double("control.cmg.kp", p.cmg.kp);
  p.cmg.kd = get_double("control.cmg.kd", p.cmg.kd);
  p.thruster.kp = get_double("control.thruster.kp", p.thruster.kp);
  p.thruster.kd = get_double("control.thruster.kd", p.thruster.kd);

  p.filter.angle_alpha = get_double("control.filter.angle_alpha", p.filter.angle_alpha);
  p.filter.rate_alpha = get_double("control.filter.rate_alpha", p.filter.rate_alpha);

  p.dead_zone.enabled = get_bool("control.dead_zone.enabled", p.dead_zone.enabled);
  p.dead_zone.angle_on_deg = get_double("control.dead_zone.angle_on_deg", p.dead_zone.angle_on_deg);
  p.dead_zone.angle_off_deg =
    get_double("control.dead_zone.angle_off_deg", p.dead_zone.angle_off_deg);
  p.dead_zone.rate_on_dps = get_double("control.dead_zone.rate_on_dps", p.dead_zone.rate_on_dps);
  p.dead_zone.rate_off_dps = get_double("control.dead_zone.rate_off_dps", p.dead_zone.rate_off_dps);

  p.unload.gain = get_double("control.unload.gain", p.unload.gain);
  p.unload.completion_threshold =
    get_double("control.unload.completion_threshold", p.unload.completion_threshold);

  p.mode.min_dwell_s = get_double("modes.min_dwell_s", p.mode.min_dwell_s);
  p.gimbal_rate_limit = get_double("actuators.gimbal_rate_limit", p.gimbal_rate_limit);
  return p;
}

plant::DisturbanceParams GncParameterBridge::read_disturbance_parameters() const
{
  plant::DisturbanceParams d;
  d.enable_gravity_gradient =
    get_bool("disturbance.enable_gravity_gradient", d.enable_gravity_gradient);
  d.enable_aero_drag = get_bool("disturbance.enable_aero_drag", d.enable_aero_drag);
  d.enable_srp = get_bool("disturbance.enable_srp", d.enable_srp);
  d.drag_coefficient = get_double("disturbance.drag_coefficient", d.drag_coefficient);
  d.frontal_area_m2 = get_double("disturbance.frontal_area_m2", d.frontal_area_m2);
  d.center_of_pressure_offset_m =
    get_double("disturbance.cop_offset_m", d.center_of_pressure_offset_m);
  d.srp_area_m2 = get_double("disturbance.srp_area_m2", d.srp_area_m2);
  d.srp_reflectivity = get_double("disturbance.srp_reflectivity", d.srp_reflectivity);
  d.srp_center_offset_m = get_double("disturbance.srp_center_offset_m", d.srp_center_offset_m);
  return d;
}

Eigen::Matrix3d GncParameterBridge::read_inertia_tensor() const
{
  const plant::Inertia fallback = plant::default_inertia();
  const double ixx = get_double("inertia.ixx", fallback.tensor()(0, 0));
  const double iyy = get_double("inertia.iyy", fallback.tensor()(1, 1));
  const double izz = get_double("inertia.izz", fallback.tensor()(2, 2));
  const double ixy = get_double("inertia.ixy", 0.0);
  const double ixz = get_double("inertia.ixz", 0.0);
  const double iyz = get_double("inertia.iyz", 0.0);

  Eigen::Matrix3d t;
  t << ixx, ixy, ixz,
    ixy, iyy, iyz,
    ixz, iyz, izz;
  return t;
}

rcl_interfaces::msg::SetParametersResult GncParameterBridge::validate(
  const std::vector<rclcpp::Parameter> & params) const
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  auto reject = [&result](const std::string & why) {
      result.successful = false;
      result.reason = why;
    };

  for (const auto & p : params) {
    const std::string & name = p.get_name();

    if (is_static_parameter(name)) {
      reject(name + " is static and only takes effect on reconfigure");
      return result;
    }

    if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      continue;
    }
    const double v = p.as_double();

    if (!std::isfinite(v)) {
      reject(name + " must be finite");
      return result;
    }

    if (name.rfind("control.cmg.", 0) == 0 || name.rfind("control.thruster.", 0) == 0 ||
      name == "control.unload.gain")
    {
      if (v < 0.0) {
        reject(name + " must be non-negative");
        return result;
      }
    }

    if (name.rfind("control.filter.", 0) == 0 && (v < 0.0 || v >= 1.0)) {
      reject(name + " must be in [0, 1)");
      return result;
    }

    if ((name.rfind("control.dead_zone.", 0) == 0 ||
      name == "modes.min_dwell_s" ||
      name == "actuators.gimbal_rate_limit" ||
      name == "control.unload.completion_threshold") && v < 0.0)
    {
      reject(name + " must be non-negative");
      return result;
    }
  }

  return result;
}
}
}
