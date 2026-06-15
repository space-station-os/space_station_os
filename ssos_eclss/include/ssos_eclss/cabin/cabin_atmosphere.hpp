#ifndef SSOS_ECLSS__CABIN__CABIN_ATMOSPHERE_HPP_
#define SSOS_ECLSS__CABIN__CABIN_ATMOSPHERE_HPP_

// Well-mixed cabin atmosphere control volume. Tracks the molar inventory of
// O2, CO2, N2 and H2O and derives pressures, composition and humidity. All
// subsystems read boundary conditions from here and write their flows back.
// No ROS, no external deps.

namespace ssos_eclss
{
namespace cabin
{

/// Gas species tracked in the cabin.
enum class Gas
{
  O2,
  CO2,
  N2,
  H2O
};

/// Volumetric/molar source-and-sink rates applied over a step [mol/s].
/// Positive adds to the cabin, negative removes.
struct GasFlows
{
  double o2{0.0};
  double co2{0.0};
  double n2{0.0};
  double h2o{0.0};
};

/// Cabin atmosphere parameters.
struct CabinParams
{
  double volume_m3;        // free cabin volume [m^3]
  double temperature_k;    // controlled cabin temperature [K]
};

/// Well-mixed cabin atmosphere.
class CabinAtmosphere
{
public:
  /// @param params cabin parameters
  CabinAtmosphere(const CabinParams & params);

  /// Initialise to a nominal ISS-like atmosphere:
  /// ~101.3 kPa total, 21% O2, ppCO2 at given ppm, 40% RH.
  void initialize_nominal(double co2_ppm = 2600.0, double relative_humidity = 0.40);

  /// Apply constant molar source/sink rates for dt seconds.
  void apply_flows(double dt, const GasFlows & flows);

  /// Directly add/remove a quantity of a species [mol] (e.g. discrete events).
  void add_moles(Gas species, double moles);

  // ---- Derived quantities ----
  double total_moles() const;
  double moles(Gas species) const;
  double total_pressure_pa() const;
  double partial_pressure_pa(Gas species) const;
  double mole_fraction(Gas species) const;
  double co2_ppm() const;
  double o2_fraction() const;
  double water_partial_pressure_pa() const { return partial_pressure_pa(Gas::H2O); }
  double relative_humidity() const;
  double dew_point_k() const;
  double temperature_k() const { return params_.temperature_k; }
  double volume_m3() const { return params_.volume_m3; }
  double total_gas_mass_kg() const;

  void set_temperature(double t_k) { params_.temperature_k = t_k; }

private:
  CabinParams params_;
  double n_o2_{0.0};
  double n_co2_{0.0};
  double n_n2_{0.0};
  double n_h2o_{0.0};

  double & ref(Gas species);
  double get(Gas species) const;
};

}  // namespace cabin
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__CABIN__CABIN_ATMOSPHERE_HPP_
