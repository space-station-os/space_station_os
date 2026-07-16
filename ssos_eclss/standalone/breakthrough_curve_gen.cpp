// Breakthrough-curve generator (no ROS). Drives a single fresh adsorbent bed
// with a constant CO2 inlet and records the normalised outlet concentration
// over time, producing the Figure 6 / Figure 12 style breakthrough curve.
//
// Usage: breakthrough_curve_gen [output.csv] [duration_min]

#include <cstdlib>
#include <fstream>
#include <iostream>

#include "ssos_eclss/ars/adsorption_isotherm.hpp"
#include "ssos_eclss/ars/bed_model.hpp"
#include "ssos_eclss/common/gas_properties.hpp"
#include "ssos_eclss/common/units.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::ars;

int main(int argc, char ** argv)
{
  const std::string out_path = (argc > 1) ? argv[1] : "breakthrough_curve.csv";
  const double duration_min = (argc > 2) ? std::atof(argv[2]) : 200.0;

  BedGeometry geom = default_adsorbent_geometry();
  geom.n_cells = 40;
  TothIsotherm co2(default_co2_on_13x());
  TothIsotherm h2o(default_h2o_on_13x());
  BedModel bed(geom, default_adsorbent_thermal(), co2, h2o,
               default_adsorbent_ldf(), /*is_desiccant=*/false);
  bed.reset(units::celsius_to_kelvin(22.0));

  const double inlet_pp_co2 = units::torr_to_pa(3.0);
  const double inlet_t = units::celsius_to_kelvin(22.0);

  BedInlet in{};
  in.temperature_k = inlet_t;
  in.pressure_pa = units::torr_to_pa(760.0);
  in.velocity_superficial = 0.4;
  in.c_co2 = gas::concentration_from_pp(inlet_pp_co2, inlet_t);
  in.c_h2o = 0.0;  // dry feed (downstream of desiccant)

  std::ofstream csv(out_path);
  csv << "time_min,outlet_co2_fraction,bed_loading_mol,mean_bed_temp_f\n";

  const double dt = 2.0;
  const int n_steps = static_cast<int>(duration_min * 60.0 / dt);
  double breakthrough_min = -1.0;
  for (int i = 0; i < n_steps; ++i) {
    const BedOutputs out = bed.step(dt, in, BedMode::ADSORBING, 0.0);
    const double frac = (in.c_co2 > 0.0) ? out.outlet_c_co2 / in.c_co2 : 0.0;
    if (i % 10 == 0) {
      csv << (i * dt / 60.0) << "," << frac << "," << out.co2_loading_mol << ","
          << units::kelvin_to_fahrenheit(out.mean_solid_temp) << "\n";
    }
    // Record 5% breakthrough onset.
    if (breakthrough_min < 0.0 && frac > 0.05) {
      breakthrough_min = i * dt / 60.0;
    }
  }
  csv.close();

  std::cout << "=== Breakthrough Curve ===\n";
  std::cout << "Output CSV:        " << out_path << "\n";
  std::cout << "5% breakthrough:   "
            << (breakthrough_min >= 0.0 ? std::to_string(breakthrough_min) + " min"
                                        : "not reached")
            << "\n";
  std::cout << "Paper onset (ref): ~140-145 min\n";
  return 0;
}
