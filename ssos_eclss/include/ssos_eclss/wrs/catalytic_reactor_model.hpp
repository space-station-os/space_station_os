#ifndef SSOS_ECLSS__WRS__CATALYTIC_REACTOR_MODEL_HPP_
#define SSOS_ECLSS__WRS__CATALYTIC_REACTOR_MODEL_HPP_

#include "ssos_eclss/wrs/wrs_parameters.hpp"

// High-temperature catalytic oxidation of trace volatile organics in the water
// processor. Conversion rises with reactor temperature (Arrhenius-style).
// No ROS, no external deps.

namespace ssos_eclss
{
namespace wrs
{

/// Catalytic reactor outputs.
struct CatalyticResult
{
  double conversion;            // fraction of organics oxidised [-]
  double outlet_voc_kg_s;       // residual volatile organics [kg/s]
};

/// Catalytic oxidation reactor.
class CatalyticReactorModel
{
public:
  explicit CatalyticReactorModel(const CatalyticReactorParams & params);

  /// Conversion at the current operating temperature [-].
  double conversion(double temperature_k) const;

  /// Treat a VOC-laden stream.
  /// @param voc_in_kg_s   inlet volatile-organics rate [kg/s]
  /// @param temperature_k reactor temperature [K]
  CatalyticResult process(double voc_in_kg_s, double temperature_k) const;

  const CatalyticReactorParams & params() const { return params_; }
  void set_params(const CatalyticReactorParams & p) { params_ = p; }

private:
  CatalyticReactorParams params_;
};

}  // namespace wrs
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__WRS__CATALYTIC_REACTOR_MODEL_HPP_
