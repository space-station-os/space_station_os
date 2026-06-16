#ifndef SSOS_ECLSS__SABATIER__SABATIER_SYSTEM_HPP_
#define SSOS_ECLSS__SABATIER__SABATIER_SYSTEM_HPP_

#include <memory>

#include "ssos_eclss/sabatier/reactor_kinetics.hpp"
#include "ssos_eclss/sabatier/sabatier_parameters.hpp"

// Sabatier system: consumes CO2 from ARS desorption and H2 from the OGS,
// producing CH4 (vented) and H2O (returned to the WRS). Closing the loop.
// Includes a lumped reactor thermal model fed by the reaction exotherm. No ROS.

namespace ssos_eclss
{
namespace sabatier
{

/// Sabatier outputs after a step.
struct SabatierResult
{
  double co2_consumed_mol_s;  // CO2 reacted [mol/s]
  double h2_consumed_mol_s;   // H2 reacted [mol/s]
  double ch4_produced_mol_s;  // CH4 produced (vented) [mol/s]
  double water_produced_mol_s;  // H2O produced (to WRS) [mol/s]
  double water_produced_kg_s;   // H2O produced [kg/s]
  double conversion;          // achieved CO2 conversion [-]
  double reaction_heat_w;     // exothermic heat released [W]
  double heater_power_w;      // thermostatic trim-heater power [W]
  double reactor_temp_k;      // reactor temperature [K]
  bool hydrogen_limited;      // true if H2 was the stoichiometric limiter
};

/// Sabatier CO2 reduction system.
class SabatierSystem
{
public:
  explicit SabatierSystem(const SabatierParameters & params = default_sabatier_parameters());

  void reset(double temperature_k);

  /// Advance the reactor by dt.
  /// @param dt          timestep [s]
  /// @param co2_in_mol_s CO2 supplied [mol/s]
  /// @param h2_in_mol_s  H2 supplied [mol/s]
  SabatierResult step(double dt, double co2_in_mol_s, double h2_in_mol_s);

  double reactor_temp_k() const { return reactor_temp_k_; }
  void set_parameters(const SabatierParameters & params);
  const SabatierParameters & parameters() const { return params_; }

private:
  SabatierParameters params_;
  std::unique_ptr<ReactorKinetics> kinetics_;
  double reactor_temp_k_{600.0};
};

}  // namespace sabatier
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__SABATIER__SABATIER_SYSTEM_HPP_
