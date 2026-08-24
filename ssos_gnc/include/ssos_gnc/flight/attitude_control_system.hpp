#ifndef SSOS_GNC__FLIGHT__ATTITUDE_CONTROL_SYSTEM_HPP_
#define SSOS_GNC__FLIGHT__ATTITUDE_CONTROL_SYSTEM_HPP_

#include <vector>

#include "ssos_gnc/flight/actuators/cmg_steering.hpp"
#include "ssos_gnc/flight/actuators/thruster_allocation.hpp"
#include "ssos_gnc/flight/control/attitude_controller.hpp"
#include "ssos_gnc/flight/estimation/attitude_estimator.hpp"
#include "ssos_gnc/flight/faults/fault_injector.hpp"
#include "ssos_gnc/flight/modes/actuation_mode_machine.hpp"

namespace ssos_gnc
{

namespace flight
{

using common::Quaternion;
using common::Vector3;
using common::Vector4;

struct VehicleSensors
{
  Vector3 gyro{Vector3::Zero()};
  Quaternion star_tracker{Quaternion::Identity()};
  bool star_tracker_valid{true};
  Vector4 gimbal{Vector4::Zero()};
};

struct AttitudeCommand
{
  Quaternion attitude_ref{Quaternion::Identity()};
  Vector3 rate_ref{Vector3::Zero()};
};

struct ControlResult
{
  Quaternion attitude_est{Quaternion::Identity()};
  Vector3 rate_est{Vector3::Zero()};
  bool estimator_coasting{false};

  Vector3 torque_cmd_total{Vector3::Zero()};
  Vector3 torque_cmg_cmd{Vector3::Zero()};
  Vector3 torque_thr_cmd{Vector3::Zero()};
  double attitude_error_rad{0.0};

  Vector4 gimbal_rate_cmd{Vector4::Zero()};
  Eigen::VectorXd thruster_duty;
  Vector3 cmg_momentum{Vector3::Zero()};

  ActuationMode mode{ActuationMode::CMG};
  bool unload_active{false};
  double cmg_momentum_frac{0.0};
  double manipulability{0.0};
  bool saturated{false};
  bool dead_zone_active{false};
  bool healthy{true};
};

class AttitudeControlSystem
{
public:
  AttitudeControlSystem();
  explicit AttitudeControlSystem(const GncParameters & params);

  ControlResult step(double dt, const VehicleSensors & sensors, const AttitudeCommand & command);

  bool configure_thrusters(const ThrusterGeometry & geometry);

  void set_parameters(const GncParameters & params);
  const GncParameters & parameters() const {return params_;}

  void reset();

  void set_unload_active(bool active) {unload_requested_ = active;}
  bool unload_active() const {return unload_active_;}

  bool unload_complete() const {return unload_complete_;}

  ModeRequestResult request_mode(ActuationMode mode) {return modes_.request_mode(mode);}
  ActuationMode mode() const {return modes_.mode();}
  const ActuationModeMachine & mode_machine() const {return modes_;}

  faults::FaultInjector & fault_injector() {return faults_;}
  const faults::HealthState & health() const {return faults_.health();}

  const ThrusterAllocation & allocation() const {return allocation_;}

private:
  GncParameters params_;

  AttitudeEstimator estimator_;
  AttitudeController controller_;
  CmgSteering steering_;
  ThrusterAllocation allocation_;
  ActuationModeMachine modes_;
  faults::FaultInjector faults_;

  bool unload_requested_{false};
  bool unload_active_{false};
  bool unload_complete_{false};
};
}  // namespace flight
}  // namespace ssos_gnc

#endif  // SSOS_GNC__FLIGHT__ATTITUDE_CONTROL_SYSTEM_HPP_
