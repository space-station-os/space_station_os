#ifndef SSOS_ECLSS__OGS__GAS_SEPARATOR_MODEL_HPP_
#define SSOS_ECLSS__OGS__GAS_SEPARATOR_MODEL_HPP_

#include "ssos_eclss/ogs/ogs_parameters.hpp"

// Rotary/static phase separator that splits product gas from entrained water.
// No ROS, no external deps.

namespace ssos_eclss
{
namespace ogs
{

/// Result of a separation.
struct SeparationResult
{
  double dry_gas_mol_s;       // gas delivered downstream [mol/s]
  double recovered_water_mol_s;  // liquid water returned to feed [mol/s]
  double carryover_water_mol_s;  // water carried over with the gas [mol/s]
};

/// Phase separator.
class GasSeparatorModel
{
public:
  explicit GasSeparatorModel(const SeparatorParams & params);

  /// Separate a wet gas stream.
  /// @param gas_mol_s   product gas molar rate [mol/s]
  /// @param water_mol_s entrained water molar rate [mol/s]
  SeparationResult separate(double gas_mol_s, double water_mol_s) const;

  const SeparatorParams & params() const { return params_; }
  void set_params(const SeparatorParams & p) { params_ = p; }

private:
  SeparatorParams params_;
};

}  // namespace ogs
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__OGS__GAS_SEPARATOR_MODEL_HPP_
