#include "ssos_sim/scenario_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace ssos_sim
{

namespace
{

const char * const kNumericFields[] = {
  "altitude_km", "orbital_velocity_mps", "atmospheric_o2_pct", "atmospheric_co2_ppm",
  "cabin_pressure_kpa", "cabin_temp_celsius", "solar_flux_w_m2",
};

std::string json_escape(const std::string & s)
{
  std::string out;
  for (char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    out += c;
  }
  return out;
}

// Flat map of scalars -> JSON object. Numbers and booleans stay unquoted.
std::string params_to_json(const YAML::Node & params, const std::string & fault_name)
{
  if (!params || params.IsNull()) {
    return "{}";
  }
  if (!params.IsMap()) {
    throw std::runtime_error("fault '" + fault_name + "': params must be a map");
  }
  std::ostringstream json;
  json << "{";
  bool first = true;
  for (const auto & kv : params) {
    if (!kv.second.IsScalar()) {
      throw std::runtime_error("fault '" + fault_name + "': param '" +
                               kv.first.as<std::string>() + "' must be a scalar");
    }
    json << (first ? "" : ", ") << "\"" << json_escape(kv.first.as<std::string>()) << "\": ";
    first = false;
    const std::string raw = kv.second.Scalar();
    bool b = false;
    double d = 0.0;
    if (kv.second.Tag() == "!") {
      // Quoted in the YAML: always a string.
      json << "\"" << json_escape(raw) << "\"";
    } else if (raw == "true" || raw == "false") {
      json << raw;
    } else if (YAML::convert<double>::decode(kv.second, d)) {
      json << raw;
    } else if (YAML::convert<bool>::decode(kv.second, b)) {
      json << (b ? "true" : "false");
    } else {
      json << "\"" << json_escape(raw) << "\"";
    }
  }
  json << "}";
  return json.str();
}

template<typename T>
std::optional<T> optional_value(const YAML::Node & node, const char * key)
{
  if (node[key]) {
    return node[key].as<T>();
  }
  return std::nullopt;
}

}  // namespace

Scenario load_scenario_file(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    throw std::runtime_error("cannot read scenario '" + path + "': " + e.what());
  }
  const YAML::Node s = root["scenario"];
  if (!s || !s.IsMap()) {
    throw std::runtime_error("scenario '" + path + "' has no top-level 'scenario' map");
  }

  Scenario scenario;
  try {
    scenario.name = s["name"] ? s["name"].as<std::string>() : "unnamed";
    scenario.duration_s = optional_value<double>(s, "duration_s");
    scenario.sim_rate_hz = optional_value<double>(s, "sim_rate_hz");
    scenario.time_scale = optional_value<double>(s, "time_scale");

    if (const YAML::Node ic = s["initial_conditions"]) {
      for (const char * field : kNumericFields) {
        if (ic[field]) {
          scenario.initial_conditions[field] = ic[field].as<double>();
        }
      }
      scenario.in_eclipse = optional_value<bool>(ic, "in_eclipse");
      scenario.orbit_phase = optional_value<double>(ic, "orbit_phase");
    }

    if (const YAML::Node faults = s["faults"]) {
      if (!faults.IsSequence()) {
        throw std::runtime_error("'faults' must be a list");
      }
      for (const auto & f : faults) {
        ScheduledFault fault;
        fault.name = f["name"] ? f["name"].as<std::string>() : "unnamed";
        if (!f["type"] || !f["target_subsystem"]) {
          throw std::runtime_error("fault '" + fault.name +
                                   "' needs 'type' and 'target_subsystem'");
        }
        fault.fault_type = f["type"].as<std::string>();
        fault.target_subsystem = f["target_subsystem"].as<std::string>();
        if (f["target_interface"]) {
          fault.target_interface = f["target_interface"].as<std::string>();
        }
        const YAML::Node trigger = f["trigger"];
        const std::string trigger_type =
          trigger && trigger["type"] ? trigger["type"].as<std::string>() : "time";
        if (trigger_type != "time" || !trigger || !trigger["at_s"]) {
          throw std::runtime_error("fault '" + fault.name +
                                   "' needs a time trigger with 'at_s'");
        }
        fault.trigger_time_s = trigger["at_s"].as<double>();
        fault.parameters_json = params_to_json(f["params"], fault.name);
        fault.duration_s = f["duration_s"] ? f["duration_s"].as<double>() : 0.0;
        scenario.faults.push_back(fault);
      }
    }
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("scenario '" + path + "': " + e.what());
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("scenario '" + path + "': " + e.what());
  }

  if (scenario.orbit_phase && (*scenario.orbit_phase < 0.0 || *scenario.orbit_phase >= 1.0)) {
    throw std::runtime_error("scenario '" + path + "': orbit_phase must be in [0, 1)");
  }
  std::stable_sort(scenario.faults.begin(), scenario.faults.end(),
    [](const ScheduledFault & a, const ScheduledFault & b) {
      return a.trigger_time_s < b.trigger_time_s;
    });
  return scenario;
}

std::string fault_description(const std::string & prefix, const std::string & label,
                              const std::string & parameters_json, double duration_s)
{
  std::ostringstream ss;
  ss << prefix << ": " << label << " (params: "
     << (parameters_json.empty() ? "{}" : parameters_json)
     << ", duration: " << std::setprecision(6) << duration_s << " s)";
  return ss.str();
}

}  // namespace ssos_sim
