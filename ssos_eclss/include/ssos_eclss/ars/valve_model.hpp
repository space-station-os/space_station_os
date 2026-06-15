#ifndef SSOS_ECLSS__ARS__VALVE_MODEL_HPP_
#define SSOS_ECLSS__ARS__VALVE_MODEL_HPP_

// Selector / re-pressurisation valve. Models flow conductance vs commanded
// position and the bed re-pressurisation transient (0 -> ~800 torr in ~15 s)
// that enables bumpless transfer between bed pairs. No ROS, no external deps.

namespace ssos_eclss
{
namespace ars
{

/// Valve parameters.
struct ValveParams
{
  double cv;                 // flow coefficient (conductance) [m^3/s/sqrt(Pa)]
  double stroke_time_s;      // full open/close stroke time [s]
  double repress_time_s;     // characteristic re-pressurisation time [s]
};

/// A two-state selector valve with finite stroke and a repressurisation model.
class ValveModel
{
public:
  explicit ValveModel(const ValveParams & params);

  /// Command the valve open fraction (0 closed, 1 open).
  void command(double target_open) { target_ = clamp01(target_open); }

  /// Advance the valve actuation by dt; the position slews toward the command.
  void update(double dt);

  /// Current open fraction [0,1].
  double position() const { return position_; }

  /// Volumetric flow [m^3/s] for a pressure drop, scaled by open fraction.
  /// Q = position * cv * sign(dP) * sqrt(|dP|)
  double flow(double delta_p_pa) const;

  /// Re-pressurise a bed from @p current_pressure toward @p target_pressure over
  /// dt seconds (first-order approach). Returns the new bed pressure [Pa].
  double repressurize(double dt, double current_pressure, double target_pressure) const;

  const ValveParams & params() const { return params_; }

private:
  static double clamp01(double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }

  ValveParams params_;
  double position_{0.0};
  double target_{0.0};
};

}  // namespace ars
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__ARS__VALVE_MODEL_HPP_
