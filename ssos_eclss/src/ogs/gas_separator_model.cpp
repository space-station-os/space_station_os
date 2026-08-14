#include "ssos_eclss/ogs/gas_separator_model.hpp"

#include <algorithm>

namespace ssos_eclss
{
namespace ogs
{

GasSeparatorModel::GasSeparatorModel(const SeparatorParams & params) : params_(params)
{}

SeparationResult GasSeparatorModel::separate(double gas_mol_s, double water_mol_s) const
{
  SeparationResult r{};
  const double eff = std::clamp(params_.efficiency, 0.0, 1.0);
  const double carry = std::clamp(params_.carryover_fraction, 0.0, 1.0);

  // Gas passes downstream (a small fraction of water is carried over).
  r.dry_gas_mol_s = gas_mol_s;
  r.carryover_water_mol_s = water_mol_s * carry;
  // The rest of the entrained water is recovered, scaled by efficiency.
  r.recovered_water_mol_s = (water_mol_s - r.carryover_water_mol_s) * eff;
  return r;
}

}  // namespace ogs
}  // namespace ssos_eclss
