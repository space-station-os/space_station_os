#ifndef SSOS_ECLSS__ARS__AIR_SAVE_PUMP_MODEL_HPP_
#define SSOS_ECLSS__ARS__AIR_SAVE_PUMP_MODEL_HPP_

// Air-save scroll pump. Before a bed is exposed to vacuum it is pumped down,
// recovering cabin air (mostly N2/O2) from the void space back to the cabin so
// it is not lost overboard. Models recovered vs lost air over the air-save step.
// No ROS, no external deps.

namespace ssos_eclss
{
namespace ars
{

/// Scroll-pump parameters.
struct AirSavePumpParams
{
  double displacement_m3s;  // volumetric pumping speed [m^3/s]
  double ultimate_pressure_pa;  // pressure at which recovery stops [Pa]
  double void_volume_m3;    // recoverable void volume in the bed [m^3]
};

/// Air-save result over a step.
struct AirSaveResult
{
  double bed_pressure_pa;     // remaining bed pressure after pumping [Pa]
  double air_recovered_mol;   // moles returned to the cabin this step [mol]
  double recovery_fraction;   // recovered / initial inventory [-]
};

/// Air-save scroll pump.
class AirSavePumpModel
{
public:
  explicit AirSavePumpModel(const AirSavePumpParams & params);

  /// Pump down the bed for one step.
  /// @param dt           timestep [s]
  /// @param bed_pressure_pa current bed pressure [Pa]
  /// @param temperature_k bed temperature [K]
  /// @return recovery result; updates pressure
  AirSaveResult pump(double dt, double bed_pressure_pa, double temperature_k) const;

  const AirSavePumpParams & params() const { return params_; }

private:
  AirSavePumpParams params_;
};

}  // namespace ars
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__ARS__AIR_SAVE_PUMP_MODEL_HPP_
