#include "ssos_gnc/nodes/thruster_geometry_loader.hpp"

#include <algorithm>
#include <map>
#include <vector>

#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

namespace ssos_gnc
{

namespace nodes
{

namespace
{
bool read_vec3(const YAML::Node & node, Eigen::Vector3d & out)
{
  if (!node || !node.IsSequence() || node.size() != 3) {return false;}
  out = Eigen::Vector3d(
    node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
  return true;
}  // namespace nodes
}  // namespace ssos_gnc

ThrusterGeometryLoader::ThrusterGeometryLoader() = default;

void ThrusterGeometryLoader::reset()
{
  geometry_ = flight::ThrusterGeometry{};
}

bool ThrusterGeometryLoader::is_thruster_link(const std::string & name)
{
  return name.rfind("thruster_", 0) == 0 || name.rfind("thr_", 0) == 0;
}

LoadResult ThrusterGeometryLoader::load_urdf(const std::string & urdf_xml)
{
  LoadResult r;

  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    r.message = "failed to parse URDF";
    return r;
  }

  std::vector<std::string> names;
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Vector3d> axes;

  for (const auto & pair : model.joints_) {
    const auto & joint = pair.second;
    if (!joint) {continue;}
    if (!is_thruster_link(joint->child_link_name)) {continue;}

    const auto & origin = joint->parent_to_joint_origin_transform;
    const Eigen::Vector3d p(origin.position.x, origin.position.y, origin.position.z);

    double qx, qy, qz, qw;
    origin.rotation.getQuaternion(qx, qy, qz, qw);
    const Eigen::Quaterniond q(qw, qx, qy, qz);
    const Eigen::Vector3d axis = q.normalized() * Eigen::Vector3d::UnitZ();

    names.push_back(joint->child_link_name);
    positions.push_back(p);
    axes.push_back(axis);
  }

  if (names.empty()) {
    r.message = "URDF parsed but contains no thruster links";
    return r;
  }

  std::vector<std::size_t> order(names.size());
  for (std::size_t i = 0; i < order.size(); ++i) {order[i] = i;}
  std::sort(
    order.begin(), order.end(),
    [&names](std::size_t a, std::size_t b) {return names[a] < names[b];});

  const auto n = static_cast<Eigen::Index>(names.size());
  geometry_.names.clear();
  geometry_.positions.resize(3, n);
  geometry_.orientations.resize(3, n);
  geometry_.max_thrust = Eigen::VectorXd::Zero(n);

  for (Eigen::Index i = 0; i < n; ++i) {
    const std::size_t src = order[static_cast<std::size_t>(i)];
    geometry_.names.push_back(names[src]);
    geometry_.positions.col(i) = positions[src];
    geometry_.orientations.col(i) = axes[src];
  }

  r.success = true;
  r.thruster_count = geometry_.names.size();
  r.message = "loaded " + std::to_string(r.thruster_count) + " thrusters from URDF";
  return r;
}

LoadResult ThrusterGeometryLoader::load_table(
  const std::string & yaml_path, const std::string & table_name)
{
  LoadResult r;

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception & e) {
    r.message = std::string("failed to read thruster table: ") + e.what();
    return r;
  }

  const YAML::Node tables = root["tables"];
  if (!tables || !tables.IsMap()) {
    r.message = "thruster table has no 'tables' mapping";
    return r;
  }

  const YAML::Node table = tables[table_name];
  if (!table || !table.IsMap()) {
    r.message = "thruster table '" + table_name + "' not found";
    return r;
  }

  std::vector<std::string> names;
  std::vector<Eigen::Vector3d> forces;
  std::vector<Eigen::Vector3d> torques;

  for (auto it = table.begin(); it != table.end(); ++it) {
    const std::string name = it->first.as<std::string>();
    const YAML::Node & entry = it->second;

    Eigen::Vector3d force = Eigen::Vector3d::Zero();
    Eigen::Vector3d torque = Eigen::Vector3d::Zero();
    if (!read_vec3(entry["force"], force)) {continue;}
    read_vec3(entry["torque"], torque);

    names.push_back(name);
    forces.push_back(force);
    torques.push_back(torque);
  }

  if (names.empty()) {
    r.message = "thruster table '" + table_name + "' contains no usable entries";
    return r;
  }

  std::vector<std::size_t> order(names.size());
  for (std::size_t i = 0; i < order.size(); ++i) {order[i] = i;}
  std::sort(
    order.begin(), order.end(),
    [&names](std::size_t a, std::size_t b) {return names[a] < names[b];});

  const auto n = static_cast<Eigen::Index>(names.size());
  geometry_.names.clear();
  geometry_.orientations.resize(3, n);
  geometry_.torque_axes.resize(3, n);
  geometry_.max_thrust = Eigen::VectorXd::Zero(n);

  for (Eigen::Index i = 0; i < n; ++i) {
    const std::size_t src = order[static_cast<std::size_t>(i)];
    const Eigen::Vector3d & f = forces[src];
    const double magnitude = f.norm();

    geometry_.names.push_back(names[src]);
    geometry_.max_thrust(i) = magnitude;
    geometry_.orientations.col(i) =
      (magnitude > 1e-9) ? Eigen::Vector3d(f / magnitude) : Eigen::Vector3d::Zero();
    geometry_.torque_axes.col(i) = torques[src];
  }

  if (geometry_.positions.cols() != n) {
    geometry_.positions = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, n);
  }

  r.success = true;
  r.thruster_count = geometry_.names.size();
  r.message = "loaded table '" + table_name + "' with " +
    std::to_string(r.thruster_count) + " thrusters";
  return r;
}
}
}
