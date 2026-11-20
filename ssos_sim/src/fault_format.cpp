#include "ssos_sim/fault_format.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace ssos_sim
{

const std::map<std::string, std::set<std::string>> & drill_catalog()
{
  static const std::map<std::string, std::set<std::string>> catalog = {
    {"crew", {"crew_metabolic_surge"}},
    {"cabin", {"cabin_leak", "o2_sensor_stuck", "chx_failure"}},
    {"ars", {"ars_scrubber_degraded", "ars_blower_failure", "ars_heater_failure"}},
    {"ogs", {"ogs_generation_failure"}},
    {"wrs", {"upa_assembly_failure", "wpa_multifiltration_breakthrough"}},
    {"sabatier", {"sabatier_reactor_failure"}},
    {"eps", {"solar_array_degraded", "array_gimbal_lock", "battery_degraded",
      "battery_low_soc", "battery_oru_failed", "battery_oru_overheat", "bcdu_fault",
      "mbsu_channel_loss", "mbsu_fault", "ddcu_fault", "ddcu_overheat", "load_spike"}},
    {"thermal", {"itcs_loop_degraded", "coldplate_conductance_loss"}},
  };
  return catalog;
}

bool is_known_drill(const std::string & target, const std::string & type)
{
  const auto it = drill_catalog().find(target);
  return it != drill_catalog().end() && it->second.count(type) > 0;
}

bool validate_drill(const std::string & target, const std::string & type,
                    const std::map<std::string, double> & params, std::string & reason)
{
  if (!is_known_drill(target, type)) {
    reason = "no drill " + type + " for subsystem " + target;
    return false;
  }
  if (params.count("clear")) {
    reason = "drills are ended with clear_faults, not a clear parameter";
    return false;
  }
  // Per-unit EPS drills: index ranges of the default topology (24 ORUs on
  // 12 channels, 4 MBSUs, 8 DDCUs).
  static const std::map<std::string, std::pair<std::string, int>> units = {
    {"battery_oru_failed", {"oru", 24}}, {"battery_oru_overheat", {"oru", 24}},
    {"bcdu_fault", {"channel", 12}}, {"mbsu_channel_loss", {"channel", 12}},
    {"mbsu_fault", {"mbsu", 4}}, {"ddcu_fault", {"ddcu", 8}}, {"ddcu_overheat", {"ddcu", 8}},
  };
  const auto u = units.find(type);
  if (target == "eps" && u != units.end()) {
    const auto it = params.find(u->second.first);
    const double index = it == params.end() ? 0.0 : it->second;
    if (index < 0.0 || index >= u->second.second || index != static_cast<int>(index)) {
      reason = u->second.first + " must be an integer in [0, " +
        std::to_string(u->second.second - 1) + "]";
      return false;
    }
  }
  return true;
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

bool parse_fault_description(const std::string & description, std::string & params_json,
                             double & duration_s)
{
  if (description.rfind("Injected fault:", 0) != 0 &&
    description.rfind("Scheduled fault:", 0) != 0)
  {
    return false;
  }
  params_json = "{}";
  duration_s = 0.0;

  const auto p = description.find("(params: ");
  if (p != std::string::npos) {
    const auto open = description.find('{', p);
    if (open != std::string::npos) {
      int depth = 0;
      for (std::size_t i = open; i < description.size(); ++i) {
        if (description[i] == '{') {
          ++depth;
        } else if (description[i] == '}' && --depth == 0) {
          params_json = description.substr(open, i - open + 1);
          break;
        }
      }
    }
  }
  const auto d = description.rfind("duration: ");
  if (d != std::string::npos) {
    duration_s = std::strtod(description.c_str() + d + 10, nullptr);
  }
  return true;
}

std::map<std::string, double> parse_fault_params(const std::string & json)
{
  std::map<std::string, double> out;
  std::size_t pos = 0;
  while (true) {
    const auto k0 = json.find('"', pos);
    if (k0 == std::string::npos) {
      break;
    }
    const auto k1 = json.find('"', k0 + 1);
    if (k1 == std::string::npos) {
      break;
    }
    const std::string key = json.substr(k0 + 1, k1 - k0 - 1);
    const auto colon = json.find(':', k1);
    if (colon == std::string::npos) {
      break;
    }
    std::size_t v = colon + 1;
    while (v < json.size() && std::isspace(static_cast<unsigned char>(json[v]))) {
      ++v;
    }
    if (v < json.size() && json[v] == '"') {
      const auto end = json.find('"', v + 1);
      pos = end == std::string::npos ? json.size() : end + 1;
      continue;
    }
    if (json.compare(v, 4, "true") == 0) {
      out[key] = 1.0;
    } else if (json.compare(v, 5, "false") == 0) {
      out[key] = 0.0;
    } else {
      char * end = nullptr;
      const double value = std::strtod(json.c_str() + v, &end);
      if (end != json.c_str() + v && std::isfinite(value)) {
        out[key] = value;
      }
    }
    pos = v + 1;
  }
  return out;
}

std::optional<InjectedFault> parse_injected_fault(const std::string & fault_type,
                                                  const std::string & description)
{
  std::string json;
  double duration = 0.0;
  if (!parse_fault_description(description, json, duration)) {
    return std::nullopt;
  }
  return InjectedFault{fault_type, parse_fault_params(json), duration};
}

void FaultTracker::apply(const InjectedFault & fault, double now_s)
{
  if (fault.type == kClearFaults) {
    clear();
    return;
  }
  active_ = fault;
  expires_s_ = fault.duration_s > 0.0 ? now_s + fault.duration_s : -1.0;
  ++revision_;
}

bool FaultTracker::expire(double now_s)
{
  if (active_ && expires_s_ >= 0.0 && now_s >= expires_s_) {
    clear();
    return true;
  }
  return false;
}

void FaultTracker::clear()
{
  if (active_) {
    active_.reset();
    expires_s_ = -1.0;
    ++revision_;
  }
}

const std::string & FaultTracker::type() const
{
  static const std::string none;
  return active_ ? active_->type : none;
}

double FaultTracker::param(const std::string & name, double fallback) const
{
  if (!active_) {
    return fallback;
  }
  const auto it = active_->params.find(name);
  return it == active_->params.end() ? fallback : it->second;
}

}  // namespace ssos_sim
