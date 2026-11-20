// Standalone EPS orbit validation (NO ROS).
//
// Runs the generation and storage chain through repeated LEO orbits with the
// same eclipse geometry as the ssos_sim default world model and prints one
// summary line per orbit: generation, battery SOC swing, load supply, the
// hottest DDCU and ORU, and how long EPS reported unhealthy. Optional CSV of
// the full time history.
//
//   eps_orbit_validation [orbits] [load_w] [csv_path]
//
// load_w scales the baseline station loads to that total.

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
  // Scale the baseline station loads to the requested total.
  const double baseline_w = eps.loads().total_requested_w();
  eps.set_demand_scale(baseline_w > 0.0 ? load_w / baseline_w : 0.0);
  std::printf("EPS orbit validation: period %.0f s, eclipse %.0f s, load %.0f W\n",
              period_s, period_s * eclipse_fraction, load_w);
  std::printf("%5s %8s %9s %8s %8s %8s %12s %9s %10s %9s %12s\n", "orbit", "gen_kWh",
              "load_kWh", "soc_min", "soc_max", "soc_end", "unserved_Wh", "beta_deg",
              "ddcu_max_C", "oru_max_C", "unhealthy_s");

  double t = 0.0;
  for (int k = 0; k < orbits; ++k) {
    double gen_wh = 0.0;
    double load_wh = 0.0;
    double unserved_wh = 0.0;
    double soc_min = 1.0;
    double soc_max = 0.0;
    double beta = 0.0;
    double ddcu_max_c = -273.0;
    double oru_max_c = -273.0;
    double unhealthy_s = 0.0;
    const int steps = static_cast<int>(period_s / dt);
    for (int i = 0; i < steps; ++i) {
      const double phase = std::fmod(t, period_s) / period_s;
      EpsInput in{};
      in.in_eclipse = phase > (1.0 - eclipse_fraction);
      in.solar_flux_w_m2 = in.in_eclipse ? 0.0 : 1361.0;
      const auto & s = eps.step(dt, in);

      gen_wh += s.available_power_w * dt / 3600.0;
      load_wh += s.load_served_w * dt / 3600.0;
      unserved_wh += s.unserved_load_w * dt / 3600.0;
      soc_min = std::min(soc_min, s.battery_soc);
      soc_max = std::max(soc_max, s.battery_soc);
      beta = s.array.beta_deg;
      for (const auto & d : s.ddcus) {
        ddcu_max_c = std::max(ddcu_max_c, d.temperature_c);
      }
      for (const auto & o : s.orus) {
        oru_max_c = std::max(oru_max_c, o.temperature_c);
      }
      unhealthy_s += s.healthy ? 0.0 : dt;
      if (csv) {
        std::fprintf(csv, "%.0f,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.5f,%.2f\n", t,
                     in.in_eclipse ? 1 : 0, s.generation_w, s.available_power_w,
                     s.load_served_w, s.battery_power_w, s.shunted_power_w, s.battery_soc,
                     s.channels.empty() ? 0.0 : s.channels.front().bus_voltage_v);
      }
      t += dt;
    }
    std::printf("%5d %8.1f %9.1f %8.4f %8.4f %8.4f %12.1f %9.1f %10.1f %9.1f %12.0f\n",
                k + 1, gen_wh / 1000.0, load_wh / 1000.0, soc_min, soc_max,
                eps.state().battery_soc, unserved_wh, beta, ddcu_max_c, oru_max_c,
                unhealthy_s);
  }

  if (csv) {
    std::fclose(csv);
    std::printf("wrote %s\n", csv_path.c_str());
  }
  return 0;
}
