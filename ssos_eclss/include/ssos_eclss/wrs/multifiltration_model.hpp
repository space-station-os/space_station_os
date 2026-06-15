#ifndef SSOS_ECLSS__WRS__MULTIFILTRATION_MODEL_HPP_
#define SSOS_ECLSS__WRS__MULTIFILTRATION_MODEL_HPP_

#include "ssos_eclss/wrs/wrs_parameters.hpp"

// Multifiltration bed model (adsorption + ion exchange) for the Water Processor.
// Tracks contaminant loading toward breakthrough and the resulting product
// water conductivity. No ROS, no external deps.

namespace ssos_eclss
{
namespace wrs
{

/// Multifiltration outputs over a step.
struct MultifiltrationResult
{
  double product_kg_s;          // treated water [kg/s]
  double product_conductivity_us;  // product conductivity [uS/cm]
  double bed_utilization;       // fraction of capacity consumed [-]
  bool broken_through;          // true once capacity is exhausted
};

/// Multifiltration bed.
class MultifiltrationModel
{
public:
  explicit MultifiltrationModel(const MultifiltrationParams & params);

  /// Reset accumulated contaminant loading.
  void reset();

  /// Treat a feed stream of given conductivity for dt seconds.
  /// @param dt                timestep [s]
  /// @param feed_kg_s         feed rate [kg/s]
  /// @param feed_conductivity_us feed conductivity [uS/cm]
  MultifiltrationResult process(double dt, double feed_kg_s,
                                double feed_conductivity_us);

  double accumulated_kg() const { return accumulated_kg_; }
  const MultifiltrationParams & params() const { return params_; }
  void set_params(const MultifiltrationParams & p) { params_ = p; }

private:
  MultifiltrationParams params_;
  double accumulated_kg_{0.0};  // contaminant mass captured [kg]
};

}  // namespace wrs
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__WRS__MULTIFILTRATION_MODEL_HPP_
