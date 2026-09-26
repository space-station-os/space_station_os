
#include <cmath>
#include <cstdio>

#include "ssos_gnc/flight/attitude_control_system.hpp"
#include "ssos_gnc/plant/disturbance_torques.hpp"
#include "ssos_gnc/plant/rigid_body.hpp"

using namespace ssos_gnc;

int run(bool momentum_conserving)
{
  flight::AttitudeControlSystem control;
  plant::RigidBody body(plant::Inertia::from_principal(1.0e8, 8.0e7, 6.0e7));
  plant::DisturbanceTorques disturbances;

  body.set_momentum_conserving(momentum_conserving);
  std::printf(
    "\n=== %s ===\n",
    momentum_conserving ?
    "momentum-conserving (physically correct)" :
    "legacy torque mode (space_station_gnc parity)");

  flight::AttitudeCommand command;
  command.attitude_ref = common::Quaternion(
    Eigen::AngleAxisd(common::deg_to_rad(20.0), Eigen::Vector3d::UnitZ()));

  plant::EnvironmentConditions env;
  env.altitude_km = 420.0;

  const double dt = 0.1;
  const int steps = 6000;

  std::printf("  t[s]   err[deg]   |h|/env    mode       manip\n");

  for (int i = 0; i < steps; ++i) {
    const plant::RigidBodyState & s = body.state();

    flight::VehicleSensors sensors;
    sensors.gyro = s.omega;
    sensors.star_tracker = s.attitude;
    sensors.star_tracker_valid = true;
    sensors.gimbal = s.gimbal;

    const flight::ControlResult r = control.step(dt, sensors, command);

    env.attitude_lvlh = s.attitude;
    const plant::DisturbanceResult dist = disturbances.compute(env, body.inertia());

    plant::RigidBodyInput input;
    input.external_torque = dist.total + r.torque_thr_cmd;
    input.gimbal_rate_cmd = r.gimbal_rate_cmd;
    input.cmg_torque_cmd = r.torque_cmg_cmd;
    body.step(dt, input);

    if (i % 600 == 0) {
      std::printf(
        "%6.1f   %8.3f   %7.4f    %-9s  %.3e\n",
        i * dt,
        common::rad_to_deg(r.attitude_error_rad),
        r.cmg_momentum_frac,
        flight::ActuationModeMachine::mode_name(r.mode),
        r.manipulability);
    }
  }

  const double final_err = common::rad_to_deg(
    common::quat_angle(common::quat_error(body.state().attitude, command.attitude_ref)));
  std::printf("\nfinal attitude error: %.4f deg\n", final_err);

  if (!std::isfinite(final_err)) {
    std::printf("FAIL: attitude diverged to non-finite\n");
    return 1;
  }
  return 0;
}

int main()
{
  int rc = run(false);
  rc |= run(true);
  return rc;
}
