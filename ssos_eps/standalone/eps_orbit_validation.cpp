// Standalone EPS orbit validation (NO ROS).
//
// Runs the generation and storage chain through repeated LEO orbits with the
// same eclipse geometry as the ssos_sim default world model and prints one
// summary line per orbit: generation, battery SOC swing, depth of discharge
// and load supply. Optional CSV of the full time history.
//
//   eps_orbit_validation [orbits] [load_w] [csv_path]

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "ssos_eps/system/eps_system.hpp"

using ssos_eps::system::EpsInput;
using ssos_eps::system::EpsSystem;

int main(int argc, char ** argv)
{
  const int orbits = argc > 1 ? std::atoi(argv[1]) : 3;
  const double load_w = argc > 2 ? std::atof(argv[2]) : 30000.0;
  const std::string csv_path = argc > 3 ? argv[3] : "";

  // ssos_sim DefaultWorldModel: 408 km, 7660 m/s, 35% of the orbit in shadow.
  const double period_s = 2.0 * 3.14159265358979323846 * (6371.0 + 408.0) * 1000.0 / 7660.0;
  const double eclipse_fraction = 0.35;
  const double dt = 1.0;

  FILE * csv = nullptr;
  if (!csv_path.empty()) {
    csv = std::fopen(csv_path.c_str(), "w");
    if (csv) {
      std::fprintf(csv, "t_s,in_eclipse,array_w,available_w,load_served_w,"
                   "battery_w,shunted_w,soc,bus_v\n");
    }
  }

  EpsSystem eps;
  std::printf("EPS orbit validation: period %.0f s, eclipse %.0f s, load %.0f W\n",
              period_s, period_s * eclipse_fraction, load_w);
  std::printf("%5s %10s %10s %8s %8s %8s %12s %10s\n", "orbit", "gen_kWh", "load_kWh",
              "soc_min", "soc_max", "soc_end", "unserved_Wh", "beta_deg");

  double t = 0.0;
  for (int k = 0; k < orbits; ++k) {
    double gen_wh = 0.0;
    double load_wh = 0.0;
    double unserved_wh = 0.0;
    double soc_min = 1.0;
    double soc_max = 0.0;
    double beta = 0.0;
    const int steps = static_cast<int>(period_s / dt);
    for (int i = 0; i < steps; ++i) {
      const double phase = std::fmod(t, period_s) / period_s;
      EpsInput in{};
      in.in_eclipse = phase > (1.0 - eclipse_fraction);
      in.solar_flux_w_m2 = in.in_eclipse ? 0.0 : 1361.0;
      in.load_demand_w = load_w;
      const auto & s = eps.step(dt, in);

      gen_wh += s.available_power_w * dt / 3600.0;
      load_wh += s.load_served_w * dt / 3600.0;
      unserved_wh += s.unserved_load_w * dt / 3600.0;
      soc_min = std::min(soc_min, s.battery_soc);
      soc_max = std::max(soc_max, s.battery_soc);
      beta = s.array.beta_deg;
      if (csv) {
        std::fprintf(csv, "%.0f,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.5f,%.2f\n", t,
                     in.in_eclipse ? 1 : 0, s.generation_w, s.available_power_w,
                     s.load_served_w, s.battery_power_w, s.shunted_power_w, s.battery_soc,
                     s.channels.empty() ? 0.0 : s.channels.front().bus_voltage_v);
      }
      t += dt;
    }
    std::printf("%5d %10.1f %10.1f %8.4f %8.4f %8.4f %12.1f %10.1f\n", k + 1,
                gen_wh / 1000.0, load_wh / 1000.0, soc_min, soc_max,
                eps.state().battery_soc, unserved_wh, beta);
  }

  if (csv) {
    std::fclose(csv);
    std::printf("wrote %s\n", csv_path.c_str());
  }
  return 0;
}
