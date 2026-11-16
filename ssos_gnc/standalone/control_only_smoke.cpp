
#include <cstdio>

#include "ssos_gnc/flight/attitude_control_system.hpp"

using namespace ssos_gnc;

int main()
{
  flight::AttitudeControlSystem control;

  flight::VehicleSensors sensors;
  flight::AttitudeCommand command;
  command.attitude_ref = common::Quaternion(
    Eigen::AngleAxisd(common::deg_to_rad(10.0), Eigen::Vector3d::UnitY()));

  const flight::ControlResult r = control.step(0.1, sensors, command);

  std::printf("mode              : %s\n", flight::ActuationModeMachine::mode_name(r.mode));
  std::printf("attitude error    : %.4f deg\n", common::rad_to_deg(r.attitude_error_rad));
  std::printf("cmg torque norm   : %.3e N.m\n", r.torque_cmg_cmd.norm());
  std::printf("gimbal rate norm  : %.3e rad/s\n", r.gimbal_rate_cmd.norm());
  std::printf("thruster ready    : %s\n", control.allocation().is_ready() ? "yes" : "no");

  if (r.torque_cmg_cmd.norm() <= 0.0) {
    std::printf("FAIL: zero control torque for a non-zero attitude error\n");
    return 1;
  }
  return 0;
}
