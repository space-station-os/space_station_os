// ARS parameter-sweep workbench (no ROS). Reproduces the paper's sensitivity
// studies by sweeping ppCO2, process flow, half-cycle time and LTL coolant
// temperature, reporting the resulting design-point CO2 removal and efficiency.
//
// Usage: parameter_sweep [output.csv]

#include <fstream>
#include <iostream>

#include "ssos_eclss/ars/four_bed_system.hpp"
#include "ssos_eclss/common/units.hpp"

using namespace ssos_eclss;

namespace
{
double removal_for(const ars::ArsParameters & p)
{
  ars::FourBedSystem fbs(p);
  return fbs.design_co2_removal_kg_day();
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::string out_path = (argc > 1) ? argv[1] : "parameter_sweep.csv";
  std::ofstream csv(out_path);
  csv << "parameter,value,co2_removal_kg_day,capture_efficiency\n";

  const ars::ArsParameters base = ars::default_ars_parameters();

  std::cout << "=== ARS Parameter Sweep ===\n";

  // Sweep ppCO2 [torr].
  for (double ppco2 = 1.0; ppco2 <= 4.0; ppco2 += 0.5) {
    ars::ArsParameters p = base;
    p.operating.inlet_ppco2_torr = ppco2;
    const double rem = removal_for(p);
    csv << "ppco2_torr," << ppco2 << "," << rem << ","
        << p.efficiency.capture_efficiency << "\n";
    std::cout << "ppCO2 = " << ppco2 << " torr -> " << rem << " kg/day\n";
  }

  // Sweep process flow [SCFM].
  for (double scfm = 14.0; scfm <= 34.0; scfm += 4.0) {
    ars::ArsParameters p = base;
    p.operating.inlet_flow_scfm = scfm;
    const double rem = removal_for(p);
    csv << "flow_scfm," << scfm << "," << rem << ","
        << p.efficiency.capture_efficiency << "\n";
    std::cout << "flow  = " << scfm << " SCFM -> " << rem << " kg/day\n";
  }

  // Sweep half-cycle adsorb time [min].
  for (double adsorb_min = 40.0; adsorb_min <= 80.0; adsorb_min += 10.0) {
    ars::ArsParameters p = base;
    p.cycle.adsorb_s = adsorb_min * units::SECONDS_PER_MINUTE;
    const double rem = removal_for(p);
    csv << "adsorb_min," << adsorb_min << "," << rem << ","
        << p.efficiency.capture_efficiency << "\n";
  }

  // Sweep LTL coolant temperature [C].
  for (double ltl_c = 4.0; ltl_c <= 16.0; ltl_c += 3.0) {
    ars::ArsParameters p = base;
    p.operating.ltl_inlet_temp_k = units::celsius_to_kelvin(ltl_c);
    const double rem = removal_for(p);
    csv << "ltl_temp_c," << ltl_c << "," << rem << ","
        << p.efficiency.capture_efficiency << "\n";
  }

  csv.close();
  std::cout << "Wrote " << out_path << "\n";
  return 0;
}
