// ARS validation workbench (no ROS). Runs the FourBedSystem at the 4BCO2 EDU
// design point, dumps a CSV time history, and compares the steady CO2 removal
// against the ICES-2021-313 target band (4.16-4.76 kg/day at 2 torr, 26 SCFM).
//
// Usage: ars_validation [output.csv] [sim_minutes]

#include <cstdlib>
#include <fstream>
#include <iostream>

#include "ssos_eclss/ars/four_bed_system.hpp"
#include "ssos_eclss/common/gas_properties.hpp"
#include "ssos_eclss/common/units.hpp"

using namespace ssos_eclss;

int main(int argc, char ** argv)
{
  const std::string out_path = (argc > 1) ? argv[1] : "ars_validation.csv";
  const double sim_minutes = (argc > 2) ? std::atof(argv[2]) : 180.0;

  ars::ArsParameters params = ars::default_ars_parameters();
  // Moderate resolution keeps the multi-hour run fast while staying faithful.
  params.desiccant_bed.n_cells = 20;
  params.adsorbent_bed.n_cells = 20;
  ars::FourBedSystem fbs(params);
  fbs.reset(units::celsius_to_kelvin(22.0));

  ars::CabinConditions cabin{};
  cabin.co2_partial_pressure_pa = units::torr_to_pa(2.0);
  cabin.h2o_partial_pressure_pa =
      gas::water_pp_from_rh(0.40, units::celsius_to_kelvin(22.0));
  cabin.temperature_k = units::celsius_to_kelvin(22.0);
  cabin.total_pressure_pa = units::torr_to_pa(760.0);

  std::ofstream csv(out_path);
  csv << "time_s,co2_removal_kg_day,scrubbed_co2_torr,system_dp_in_h2o,"
         "max_bed_temp_f,precooler_exit_f,blower_flow_scfm,adsorbing_train\n";

  const double dt = 2.0;
  const int n_steps = static_cast<int>(sim_minutes * 60.0 / dt);
  double sum_removal = 0.0;
  int sample_count = 0;

  for (int i = 0; i < n_steps; ++i) {
    const ars::ArsResult r = fbs.step(dt, cabin);
    const double t = i * dt;
    if (i % 15 == 0) {
      csv << t << "," << r.co2_removal_rate_kg_day << ","
          << units::pa_to_torr(r.scrubbed_co2_pp_pa) << ","
          << units::pa_to_in_h2o(r.system_pressure_drop_pa) << ","
          << units::kelvin_to_fahrenheit(r.max_bed_temp_k) << ","
          << units::kelvin_to_fahrenheit(r.precooler_exit_temp_k) << ","
          << r.blower_flow_scfm << "," << r.adsorbing_train << "\n";
    }
    // Average over the second half (cyclic steady-ish state).
    if (i > n_steps / 2) {
      sum_removal += r.co2_removal_rate_kg_day;
      ++sample_count;
    }
  }
  csv.close();

  const double mean_removal = (sample_count > 0) ? sum_removal / sample_count : 0.0;
  const double design = fbs.design_co2_removal_kg_day();

  std::cout << "=== ARS Validation (ICES-2021-313) ===\n";
  std::cout << "Output CSV:            " << out_path << "\n";
  std::cout << "Design CO2 removal:    " << design << " kg/day\n";
  std::cout << "Mean transient removal:" << mean_removal << " kg/day\n";
  std::cout << "Paper target band:     4.16 - 4.76 kg/day\n";
  const bool in_band = (design >= 4.16 && design <= 4.76);
  std::cout << "Design in band:        " << (in_band ? "YES" : "NO") << "\n";
  return in_band ? 0 : 1;
}
