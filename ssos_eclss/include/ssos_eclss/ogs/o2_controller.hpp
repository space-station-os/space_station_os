#ifndef SSOS_ECLSS__OGS__O2_CONTROLLER_HPP_
#define SSOS_ECLSS__OGS__O2_CONTROLLER_HPP_

// Cabin ppO2 control for the OGS: a PI loop on the measured O2 partial
// pressure sets the stack current. The integral term carries the steady
// current (crew consumption plus leakage), so it is clamped to the stack's
// current range to avoid wind-up. No ROS.

namespace ssos_eclss
{
namespace ogs
{

struct O2ControllerParams
{
  double setpoint_kpa;        // cabin ppO2 target
  double kp_a_per_kpa;        // proportional gain
  double integral_time_s;     // integral time constant
  double min_current_a;
  double max_current_a;
};

O2ControllerParams default_o2_controller_params();

class O2PressureController
{
public:
  explicit O2PressureController(
    const O2ControllerParams & params = default_o2_controller_params());

  void set_params(const O2ControllerParams & params) {params_ = params;}
  const O2ControllerParams & params() const {return params_;}

  /// Start from a known current with no error (bumpless start).
  void reset(double current_a);
  /// Stack current for the next dt seconds.
  double update(double dt, double measured_kpa);
  double current() const {return current_;}

private:
  O2ControllerParams params_;
  double integral_a_{0.0};
  double current_{0.0};
};

}  // namespace ogs
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__OGS__O2_CONTROLLER_HPP_
