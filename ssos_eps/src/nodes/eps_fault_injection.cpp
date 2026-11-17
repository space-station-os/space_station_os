#include "ssos_eps/nodes/eps_fault_injection.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>

namespace ssos_eps
{
namespace nodes
{

namespace
{

struct TypeInfo
{
  const char * type;
  const char * index;          // per-unit index parameter, or ""
  const char * value;          // main value parameter, or ""
  double default_value;
};

const std::vector<TypeInfo> & type_table()
{
  static const std::vector<TypeInfo> table = {
    {"solar_array_degraded", "", "factor", 0.3},
    {"battery_degraded", "", "state_of_health", 0.5},
    {"battery_low_soc", "", "soc", 0.15},
    {"battery_oru_failed", "oru", "", 0.0},
    {"bcdu_fault", "channel", "", 0.0},
    {"mbsu_channel_loss", "channel", "", 0.0},
    {"mbsu_fault", "mbsu", "", 0.0},
    {"ddcu_fault", "ddcu", "", 0.0},
    {"load_spike", "", "demand_scale", 2.0},
    {"clear_faults", "", "", 0.0},
  };
  return table;
}

const TypeInfo * find_type(const std::string & type)
{
  for (const auto & t : type_table()) {
    if (type == t.type) {
      return &t;
    }
  }
  return nullptr;
}

}  // namespace

const std::vector<std::string> & EpsFaultInjection::supported_types()
{
  static const std::vector<std::string> names = [] {
      std::vector<std::string> out;
      for (const auto & t : type_table()) {
        out.push_back(t.type);
      }
      return out;
    }();
  return names;
}

std::string EpsFaultInjection::index_param(const std::string & type)
{
  const auto * info = find_type(type);
  return info != nullptr ? info->index : "";
}

bool EpsFaultInjection::parse_description(const std::string & description,
                                          std::string & params_json, double & duration_s)
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

std::map<std::string, double> EpsFaultInjection::parse_params(const std::string & json)
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
      // String value: skip it.
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
    const auto next = json.find_first_of(",}", v);
    pos = next == std::string::npos ? json.size() : next + 1;
  }
  return out;
}

bool EpsFaultInjection::apply(const std::string & type,
                              const std::map<std::string, double> & params,
                              double duration_s, double now_s, std::string & message)
{
  const auto * info = find_type(type);
  if (info == nullptr) {
    message = "unsupported EPS fault type '" + type + "'";
    return false;
  }

  if (type == "clear_faults") {
    const bool changed = !active_.empty();
    active_.clear();
    revision_ += changed ? 1 : 0;
    message = "cleared all injected EPS faults";
    return true;
  }

  std::string key = type;
  int index = -1;
  if (*info->index != '\0') {
    const auto it = params.find(info->index);
    index = it != params.end() ? static_cast<int>(std::lround(it->second)) : 0;
    key += ":" + std::to_string(index);
  }

  const auto clear = params.find("clear");
  if (clear != params.end() && clear->second != 0.0) {
    const auto before = active_.size();
    active_.erase(std::remove_if(active_.begin(), active_.end(),
      [&](const InjectedFault & f) {
        return f.type == type && (index < 0 || f.key == key);
      }), active_.end());
    revision_ += active_.size() != before ? 1 : 0;
    message = "cleared " + key;
    return true;
  }

  if (type == "battery_low_soc") {
    const auto it = params.find(info->value);
    pending_soc_ = std::clamp(it != params.end() ? it->second : info->default_value, 0.0, 1.0);
    ++revision_;
    message = "battery state of charge set to " + std::to_string(*pending_soc_);
    return true;
  }

  InjectedFault fault;
  fault.key = key;
  fault.type = type;
  fault.params = params;
  if (*info->value != '\0' && fault.params.count(info->value) == 0) {
    fault.params[info->value] = info->default_value;
  }
  if (index >= 0) {
    fault.params[info->index] = index;
  }
  fault.expires_s = duration_s > 0.0 ? now_s + duration_s : -1.0;

  const auto existing = std::find_if(active_.begin(), active_.end(),
      [&key](const InjectedFault & f) {return f.key == key;});
  if (existing != active_.end()) {
    *existing = fault;
  } else {
    active_.push_back(fault);
  }
  ++revision_;
  message = "injected " + key + (duration_s > 0.0 ?
    " for " + std::to_string(static_cast<int>(duration_s)) + " s" : " until cleared");
  return true;
}

bool EpsFaultInjection::expire(double now_s)
{
  const auto before = active_.size();
  active_.erase(std::remove_if(active_.begin(), active_.end(),
    [now_s](const InjectedFault & f) {return f.expires_s >= 0.0 && now_s >= f.expires_s;}),
    active_.end());
  if (active_.size() != before) {
    ++revision_;
    return true;
  }
  return false;
}

void EpsFaultInjection::clear()
{
  if (!active_.empty() || pending_soc_) {
    ++revision_;
  }
  active_.clear();
  pending_soc_.reset();
}

double EpsFaultInjection::param_or(const InjectedFault & f, const std::string & name,
                                   double fallback) const
{
  const auto it = f.params.find(name);
  return it != f.params.end() ? it->second : fallback;
}

std::set<int> EpsFaultInjection::indices(const std::string & type) const
{
  std::set<int> out;
  const std::string idx = index_param(type);
  for (const auto & f : active_) {
    if (f.type == type && !idx.empty()) {
      out.insert(static_cast<int>(std::lround(param_or(f, idx, 0.0))));
    }
  }
  return out;
}

double EpsFaultInjection::array_factor() const
{
  double factor = 1.0;
  for (const auto & f : active_) {
    if (f.type == "solar_array_degraded") {
      factor = std::min(factor, std::clamp(param_or(f, "factor", 0.3), 0.0, 1.0));
    }
  }
  return factor;
}

double EpsFaultInjection::health_factor() const
{
  double factor = 1.0;
  for (const auto & f : active_) {
    if (f.type == "battery_degraded") {
      factor = std::min(factor, std::clamp(param_or(f, "state_of_health", 0.5), 0.01, 1.0));
    }
  }
  return factor;
}

double EpsFaultInjection::demand_scale() const
{
  double scale = 1.0;
  bool any = false;
  for (const auto & f : active_) {
    if (f.type == "load_spike") {
      const double s = std::clamp(param_or(f, "demand_scale", 2.0), 0.0, 10.0);
      scale = any ? std::max(scale, s) : s;
      any = true;
    }
  }
  return scale;
}

std::optional<double> EpsFaultInjection::take_pending_soc()
{
  auto soc = pending_soc_;
  pending_soc_.reset();
  return soc;
}

}  // namespace nodes
}  // namespace ssos_eps
