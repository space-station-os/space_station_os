#ifndef SSOS_ECLSS__WRS__WRS_PARAMETERS_HPP_
#define SSOS_ECLSS__WRS__WRS_PARAMETERS_HPP_

// Parameters for the Water Recovery System (urine processor + water processor).
// SI units. Factory functions provide ISS WRS-class defaults.

namespace ssos_eclss
{
namespace wrs
{

/// Vapor Compression Distillation (Urine Processor Assembly) parameters.
/// The UPA recovers ~85-87% of water from US-pretreated urine (target 90%).
/// The optional Brine Processor Assembly (BPA) further dewaters the UPA brine,
/// raising the total urine water recovery to ~98% (ICES-2023-097).
struct DistillationParams
{
  double recovery_fraction;       // UPA water recovered from urine [-]
  double specific_energy_j_kg;    // electrical energy per kg distillate [J/kg]
  double max_throughput_kg_s;     // maximum processing rate [kg/s]
  double brine_solids_fraction;   // dissolved solids that stay in brine [-]
  bool brine_processor_enabled;   // BPA installed and processing brine
  double brine_recovery_fraction; // BPA water recovered from UPA brine [-]
};

/// Multifiltration (Water Processor Assembly) parameters.
struct MultifiltrationParams
{
  double bed_capacity_kg;       // contaminant capacity before breakthrough [kg]
  double removal_efficiency;    // fraction of contaminant removed [-]
  double inlet_conductivity_us; // typical feed conductivity [uS/cm]
};

/// Catalytic reactor (volatile removal) parameters.
struct CatalyticReactorParams
{
  double operating_temp_k;      // reactor temperature [K]
  double activation_temp_k;     // characteristic temperature for conversion [K]
  double max_conversion;        // asymptotic conversion [-]
};

/// Complete WRS parameter set.
struct WrsParameters
{
  DistillationParams distillation;
  MultifiltrationParams multifiltration;
  CatalyticReactorParams catalytic;
};

DistillationParams default_distillation_params();
MultifiltrationParams default_multifiltration_params();
CatalyticReactorParams default_catalytic_params();
WrsParameters default_wrs_parameters();

}  // namespace wrs
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__WRS__WRS_PARAMETERS_HPP_
