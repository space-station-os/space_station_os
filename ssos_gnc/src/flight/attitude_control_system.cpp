#include "ssos_gnc/flight/attitude_control_system.hpp"

namespace ssos_gnc
{

namespace flight
{

AttitudeControlSystem::AttitudeControlSystem()
: params_(default_gnc_parameters()),
  controller_(params_),
  modes_(params_.mode)
{
  steering_.set_rate_limit(params_.gimbal_rate_limit);
}  // namespace flight

AttitudeControlSystem::AttitudeControlSystem(const GncParameters & params)
: params_(params),
  controller_(params),
  modes_(params.mode)
{
  steering_.set_rate_limit(params_.gimbal_rate_limit);
}  // namespace ssos_gnc

void AttitudeControlSystem::set_parameters(const GncParameters & params)
{
  params_ = params;
  controller_.set_parameters(params);
  modes_.set_parameters(params.mode);
  steering_.set_rate_limit(params.gimbal_rate_limit);
}

void AttitudeControlSystem::reset()
{
  estimator_.reset();
  controller_.reset();
  modes_.reset();
  faults_.reset();
  unload_requested_ = false;
  unload_active_ = false;
  unload_complete_ = false;
}

bool AttitudeControlSystem::configure_thrusters(const ThrusterGeometry & geometry)
{
  return allocation_.configure(geometry);
}

ControlResult AttitudeControlSystem::step(
  double dt, const VehicleSensors & sensors, const AttitudeCommand & command)
{
  ControlResult out;

  faults_.update(dt);
  const faults::HealthState & health = faults_.health();
  out.healthy = health.all_healthy();

  EstimatorInput est_in;
  est_in.body_rate = sensors.gyro;
  est_in.attitude = sensors.star_tracker;
  est_in.attitude_valid = sensors.star_tracker_valid;
  const EstimatorOutput est = estimator_.update(dt, est_in);

  out.attitude_est = est.attitude;
  out.rate_est = est.body_rate;
  out.estimator_coasting = est.coasting;

  ModeInputs mode_in;
  mode_in.cmg_healthy = health.cmg_healthy;
  mode_in.thruster_healthy = health.thruster_healthy && allocation_.is_ready();
  modes_.update(dt, mode_in);
  out.mode = modes_.mode();

  const AttitudeError error =
    AttitudeController::compute_error(est.attitude, command.attitude_ref, est.body_rate);
  out.attitude_error_rad = error.angle_rad;

  const Vector3 h = steering_.momentum(sensors.gimbal);
  out.cmg_momentum = h;
  out.cmg_momentum_frac = steering_.momentum_fraction(sensors.gimbal);

  const double dump_norm = (params_.unload.gain * h).norm();
  unload_complete_ = dump_norm <= params_.unload.completion_threshold;
  unload_active_ = unload_requested_ && !unload_complete_;
  out.unload_active = unload_active_;

  ControlTorques torques;
  if (out.mode == ActuationMode::CMG) {
    torques = controller_.compute_cmg(error, h, unload_active_);
  } else {
    torques = controller_.compute_thruster(error);
  }

  out.torque_cmg_cmd = torques.cmg;
  out.torque_thr_cmd = torques.thruster;
  out.torque_cmd_total = torques.cmg + torques.thruster;
  out.dead_zone_active = torques.dead_zone_active;

  if (out.mode == ActuationMode::CMG && health.cmg_capability > 0.0) {
    const Vector3 effective = torques.cmg * health.cmg_capability;
    const SteeringResult steer = steering_.solve(effective, sensors.gimbal, est.body_rate);
    out.gimbal_rate_cmd = steer.gimbal_rate;
    out.manipulability = steer.manipulability;
    out.saturated = out.saturated || steer.rate_limited;
  } else {
    out.manipulability = steering_.kinematics().manipulability(sensors.gimbal);
  }

  if (allocation_.is_ready() && out.torque_thr_cmd.squaredNorm() > 0.0) {
    const Eigen::VectorXd thrust = allocation_.body_to_thruster(out.torque_thr_cmd);
    out.thruster_duty = allocation_.thrust_to_duty(thrust);
    out.saturated = out.saturated || allocation_.last_command_saturated();
  } else if (allocation_.is_ready()) {
    out.thruster_duty = Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(allocation_.thruster_count()));
  }

  return out;
}
}
}
