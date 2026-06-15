#ifndef SSOS_ECLSS__WRS__DISTILLATION_MODEL_HPP_
#define SSOS_ECLSS__WRS__DISTILLATION_MODEL_HPP_

#include "ssos_eclss/wrs/wrs_parameters.hpp"

// Vapor Compression Distillation model for urine processing: evaporation,
// vapour compression and condensation, producing distillate and brine. No ROS.

namespace ssos_eclss
{
namespace wrs
{

/// Distillation outputs over a step.
struct DistillationResult
{
  double distillate_kg_s;   // recovered water [kg/s]
  double brine_kg_s;        // concentrated brine reject [kg/s]
  double energy_w;          // electrical power consumed [W]
  double recovery_fraction; // achieved recovery [-]
};

/// Vapor Compression Distillation (Urine Processor Assembly).
class DistillationModel
{
public:
  explicit DistillationModel(const DistillationParams & params);

  /// Process a urine/wastewater feed.
  /// @param feed_kg_s urine feed rate [kg/s]
  DistillationResult process(double feed_kg_s) const;

  const DistillationParams & params() const { return params_; }
  void set_params(const DistillationParams & p) { params_ = p; }

private:
  DistillationParams params_;
};

}  // namespace wrs
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__WRS__DISTILLATION_MODEL_HPP_
