#ifndef SSOS_GNC__FLIGHT__ACTUATORS__THRUSTER_ALLOCATION_HPP_
#define SSOS_GNC__FLIGHT__ACTUATORS__THRUSTER_ALLOCATION_HPP_

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ssos_gnc/common/linalg.hpp"
#include "ssos_gnc/common/units.hpp"

namespace ssos_gnc
{

namespace flight
{

struct ThrusterTableEntry
{
  std::string name;
  Eigen::VectorXd duty;
};

struct ThrusterGeometry
{
  Eigen::Matrix<double, 3, Eigen::Dynamic> positions;
  Eigen::Matrix<double, 3, Eigen::Dynamic> orientations;
  Eigen::VectorXd max_thrust;
  std::vector<std::string> names;
  std::vector<ThrusterTableEntry> tables;

  Eigen::Matrix<double, 3, Eigen::Dynamic> torque_axes;

  std::size_t count() const {return names.size();}
  bool empty() const {return names.empty();}
  bool has_explicit_torque() const
  {
    return torque_axes.cols() == static_cast<Eigen::Index>(names.size()) && !names.empty();
  }

  bool is_valid() const;
};

class ThrusterAllocation
{
public:
  ThrusterAllocation();

  bool configure(const ThrusterGeometry & geometry);

  bool is_ready() const {return ready_;}
  std::size_t thruster_count() const {return geometry_.count();}

  const Eigen::MatrixXd & allocation_matrix() const {return allocation_;}

  Eigen::VectorXd body_to_thruster(const Eigen::Vector3d & torque_cmd) const;

  Eigen::Vector3d thruster_to_body(const Eigen::VectorXd & thrust) const;

  Eigen::VectorXd thrust_to_duty(const Eigen::VectorXd & thrust) const;

  bool last_command_saturated() const {return saturated_;}

  bool duty_from_table(const std::string & table_name, Eigen::VectorXd & out) const;

private:
  ThrusterGeometry geometry_;
  Eigen::MatrixXd allocation_;
  Eigen::MatrixXd inverse_allocation_;
  bool ready_{false};
  mutable bool saturated_{false};
};
}  // namespace flight
}  // namespace ssos_gnc

#endif  // SSOS_GNC__FLIGHT__ACTUATORS__THRUSTER_ALLOCATION_HPP_
