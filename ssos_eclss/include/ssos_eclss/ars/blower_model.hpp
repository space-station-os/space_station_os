#ifndef SSOS_ECLSS__ARS__BLOWER_MODEL_HPP_
#define SSOS_ECLSS__ARS__BLOWER_MODEL_HPP_

// Process-air blower model. A quadratic fan curve dP = f(RPM, Q) is intersected
// with the system resistance curve (Ergun-dominated) to find the operating
// point. Supports constant-RPM and constant-flow control. No ROS, no deps.

namespace ssos_eclss
{
namespace ars
{

/// Blower (fan) parameters. Fan curve: dP = a0*(N/Nref)^2 - a2*Q^2
/// where Q is volumetric flow [m^3/s], N is speed [rpm].
struct BlowerParams
{
  double rated_rpm;        // reference speed [rpm]
  double shutoff_dp;       // dead-head head at rated rpm [Pa]
  double curve_quad;       // quadratic flow coefficient a2 [Pa/(m^3/s)^2]
  double max_rpm;          // mechanical speed limit [rpm]
};

/// Control mode.
enum class BlowerControl
{
  CONSTANT_RPM,
  CONSTANT_FLOW
};

/// Operating point.
struct BlowerResult
{
  double flow_m3s;     // delivered volumetric flow [m^3/s]
  double delta_p;      // developed pressure rise [Pa]
  double rpm;          // operating speed [rpm]
  double power_w;      // shaft power estimate [W]
};

/// Centrifugal/regenerative process blower.
class BlowerModel
{
public:
  explicit BlowerModel(const BlowerParams & params);

  /// Fan head [Pa] at a given speed and flow.
  double fan_head(double rpm, double flow_m3s) const;

  /// Solve the operating point against a system resistance R such that
  /// dP_system(Q) = system_resistance * Q^2 (quadratic, Ergun-like turbulent).
  /// @param system_resistance [Pa/(m^3/s)^2]
  /// @param control control mode
  /// @param setpoint target rpm (CONSTANT_RPM) or target flow [m^3/s] (CONSTANT_FLOW)
  BlowerResult solve(double system_resistance, BlowerControl control,
                     double setpoint) const;

  const BlowerParams & params() const { return params_; }

private:
  BlowerParams params_;
};

}  // namespace ars
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__ARS__BLOWER_MODEL_HPP_
