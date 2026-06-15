#ifndef SSOS_ECLSS__ARS__PRECOOLER_MODEL_HPP_
#define SSOS_ECLSS__ARS__PRECOOLER_MODEL_HPP_

// Air-to-Low-Temperature-Loop (LTL) precooler heat exchanger using the
// epsilon-NTU method. Cools the process air before the desiccant beds.
// No ROS, no external deps.

namespace ssos_eclss
{
namespace ars
{

/// Precooler design parameters.
struct PrecoolerParams
{
  double ua;              // overall conductance U*A [W/K]
  double air_mass_flow;   // process air mass flow [kg/s]
  double coolant_mass_flow;  // LTL coolant mass flow [kg/s]
  double air_cp;          // air specific heat [J/(kg*K)]
  double coolant_cp;      // coolant (water) specific heat [J/(kg*K)]
};

/// Precooler outputs.
struct PrecoolerResult
{
  double air_outlet_temp;     // cooled air temperature [K]
  double coolant_outlet_temp; // warmed coolant temperature [K]
  double heat_duty;           // heat removed from air [W]
  double effectiveness;       // epsilon-NTU effectiveness [-]
};

/// Air-to-LTL precooler.
class PrecoolerModel
{
public:
  explicit PrecoolerModel(const PrecoolerParams & params);

  /// Compute steady-state outlet conditions.
  /// @param air_inlet_temp     process air inlet temperature [K]
  /// @param coolant_inlet_temp LTL coolant inlet temperature [K]
  PrecoolerResult solve(double air_inlet_temp, double coolant_inlet_temp) const;

  void set_params(const PrecoolerParams & p) { params_ = p; }
  const PrecoolerParams & params() const { return params_; }

private:
  PrecoolerParams params_;
};

}  // namespace ars
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__ARS__PRECOOLER_MODEL_HPP_
