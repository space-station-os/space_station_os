#ifndef SSOS_SIM__SCENARIO_LOADER_HPP_
#define SSOS_SIM__SCENARIO_LOADER_HPP_

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "ssos_sim/fault_format.hpp"

// Scenario files (scenarios/*.yaml) describe a simulation run: duration,
// rate, initial conditions and a time-ordered fault schedule. Parsing has no
// ROS dependency so it can be tested on its own.
//
//   scenario:
//     name: <string>
//     duration_s: <s>            # optional
//     sim_rate_hz: <Hz>          # optional
//     time_scale: <x>            # optional
//     initial_conditions:        # optional; any WorldState field plus
//       orbit_phase: <0..1>      #   the starting position in the orbit
//     faults:
//       - name: <string>
//         type: <fault type>
//         target_subsystem: <name>
//         target_interface: <topic>   # optional
//         trigger: {type: time, at_s: <s>}
//         params: {<key>: <scalar>, ...}   # optional
//         duration_s: <s>            # <= 0 = until cleared

namespace ssos_sim
{

/// A scheduled fault loaded from a scenario file.
struct ScheduledFault
{
  std::string name;
  std::string fault_type;
  std::string target_subsystem;
  std::string target_interface;
  double trigger_time_s{0.0};
  std::string parameters_json{"{}"};
  double duration_s{0.0};
  bool fired{false};
};

struct Scenario
{
  std::string name;
  std::optional<double> duration_s;
  std::optional<double> sim_rate_hz;
  std::optional<double> time_scale;
  std::optional<double> orbit_phase;
  std::optional<bool> in_eclipse;
  std::map<std::string, double> initial_conditions;  // numeric WorldState fields
  std::vector<ScheduledFault> faults;                 // sorted by trigger time
};

/// Parse a scenario file. Throws std::runtime_error with a readable message
/// when the file is missing or malformed.
Scenario load_scenario_file(const std::string & path);

}  // namespace ssos_sim

#endif  // SSOS_SIM__SCENARIO_LOADER_HPP_
