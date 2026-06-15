#ifndef SSOS_ECLSS__WRS__WATER_RECOVERY_SYSTEM_HPP_
#define SSOS_ECLSS__WRS__WATER_RECOVERY_SYSTEM_HPP_

#include <memory>

#include "ssos_eclss/wrs/catalytic_reactor_model.hpp"
#include "ssos_eclss/wrs/distillation_model.hpp"
#include "ssos_eclss/wrs/multifiltration_model.hpp"
#include "ssos_eclss/wrs/wrs_parameters.hpp"

// Top-level Water Recovery System. Inputs urine and humidity condensate; the
// urine is distilled (UPA) and combined with condensate, then polished by the
// catalytic reactor + multifiltration (WPA) into potable water. Tracks product
// water conductivity as the quality metric. No ROS.

namespace ssos_eclss
{
namespace wrs
{

/// WRS outputs after a step.
struct WrsResult
{
  double potable_water_kg_s;        // potable water produced [kg/s]
  double brine_kg_s;                // urine brine reject [kg/s]
  double product_conductivity_us;   // product water conductivity [uS/cm]
  double overall_recovery;          // recovered / input water [-]
  double power_w;                   // total electrical power [W]
  double voc_conversion;            // catalytic VOC conversion [-]
  bool potable_in_spec;             // conductivity below the potable limit
  bool multifiltration_broken_through;
};

/// Water Recovery System orchestrator.
class WaterRecoverySystem
{
public:
  explicit WaterRecoverySystem(const WrsParameters & params = default_wrs_parameters());

  void reset();

  /// Advance the WRS by dt.
  /// @param dt            timestep [s]
  /// @param urine_kg_s    urine feed [kg/s]
  /// @param condensate_kg_s humidity condensate feed [kg/s]
  /// @param voc_kg_s      trace volatile organics in feed [kg/s]
  WrsResult step(double dt, double urine_kg_s, double condensate_kg_s,
                 double voc_kg_s);

  void set_parameters(const WrsParameters & params);
  const WrsParameters & parameters() const { return params_; }

private:
  WrsParameters params_;
  std::unique_ptr<DistillationModel> distillation_;
  std::unique_ptr<MultifiltrationModel> multifiltration_;
  std::unique_ptr<CatalyticReactorModel> catalytic_;
};

}  // namespace wrs
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__WRS__WATER_RECOVERY_SYSTEM_HPP_
