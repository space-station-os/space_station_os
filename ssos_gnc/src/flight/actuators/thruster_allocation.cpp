#include "ssos_gnc/flight/actuators/thruster_allocation.hpp"

#include <algorithm>

namespace ssos_gnc
{

namespace flight
{

bool ThrusterGeometry::is_valid() const
{
  const auto n = static_cast<Eigen::Index>(names.size());
  if (n == 0) {return false;}
  if (max_thrust.size() != n) {return false;}

  if (has_explicit_torque()) {return true;}
  return positions.cols() == n && orientations.cols() == n;
}  // namespace flight

ThrusterAllocation::ThrusterAllocation() = default;

bool ThrusterAllocation::configure(const ThrusterGeometry & geometry)
{
  ready_ = false;
  if (!geometry.is_valid()) {return false;}

  geometry_ = geometry;
  const auto n = static_cast<Eigen::Index>(geometry_.count());

  allocation_.resize(3, n);
  if (geometry_.has_explicit_torque()) {
    allocation_ = geometry_.torque_axes;
  } else {
    for (Eigen::Index i = 0; i < n; ++i) {
      const Eigen::Vector3d r = geometry_.positions.col(i);
      const Eigen::Vector3d f = geometry_.orientations.col(i).normalized();
      allocation_.col(i) = r.cross(f);
    }
  }

  inverse_allocation_ = common::pseudo_inverse<Eigen::MatrixXd>(allocation_);

  ready_ = true;
  return true;
}  // namespace ssos_gnc

Eigen::VectorXd ThrusterAllocation::body_to_thruster(const Eigen::Vector3d & torque_cmd) const
{
  saturated_ = false;
  if (!ready_) {return Eigen::VectorXd();}

  Eigen::VectorXd thrust = inverse_allocation_ * torque_cmd;

  for (Eigen::Index i = 0; i < thrust.size(); ++i) {
    if (std::abs(thrust(i)) > geometry_.max_thrust(i)) {
      saturated_ = true;
    }
  }
  return thrust;
}

Eigen::Vector3d ThrusterAllocation::thruster_to_body(const Eigen::VectorXd & thrust) const
{
  if (!ready_ || thrust.size() != allocation_.cols()) {
    return Eigen::Vector3d::Zero();
  }
  return allocation_ * thrust;
}

Eigen::VectorXd ThrusterAllocation::thrust_to_duty(const Eigen::VectorXd & thrust) const
{
  Eigen::VectorXd duty = Eigen::VectorXd::Zero(thrust.size());
  if (!ready_ || thrust.size() != static_cast<Eigen::Index>(geometry_.count())) {
    return duty;
  }

  for (Eigen::Index i = 0; i < thrust.size(); ++i) {
    const double max_t = geometry_.max_thrust(i);
    if (max_t <= 0.0) {continue;}
    duty(i) = common::clamp(thrust(i) / max_t, 0.0, 1.0);
  }
  return duty;
}

bool ThrusterAllocation::duty_from_table(
  const std::string & table_name, Eigen::VectorXd & out) const
{
  for (const auto & entry : geometry_.tables) {
    if (entry.name == table_name) {
      out = entry.duty;
      return true;
    }
  }
  return false;
}
}
}
