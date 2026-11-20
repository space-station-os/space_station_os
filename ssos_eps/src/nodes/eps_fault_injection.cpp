#include "ssos_eps/nodes/eps_fault_injection.hpp"

#include "ssos_sim/fault_format.hpp"

#include <algorithm>
#include <cmath>

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
    {"array_gimbal_lock", "", "", 0.0},
    {"battery_degraded", "", "state_of_health", 0.5},
    {"battery_low_soc", "", "soc", 0.15},
    {"battery_oru_failed", "oru", "", 0.0},
    {"battery_oru_overheat", "oru", "heat_w", 800.0},
    {"bcdu_fault", "channel", "", 0.0},
    {"mbsu_channel_loss", "channel", "", 0.0},
    {"mbsu_fault", "mbsu", "", 0.0},
    {"ddcu_fault", "ddcu", "", 0.0},
    {"ddcu_overheat", "ddcu", "heat_w", 400.0},
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
  return ssos_sim::parse_fault_description(description, params_json, duration_s);
}

std::map<std::string, double> EpsFaultInjection::parse_params(const std::string & json)
{
  return ssos_sim::parse_fault_params(json);
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
    clear();
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
  if (type == "battery_low_soc") {
    pending_soc_ = std::clamp(fault.params[info->value], 0.0, 1.0);
  }

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

bool EpsFaultInjection::has(const std::string & type) const
{
  return std::any_of(active_.begin(), active_.end(),
           [&type](const InjectedFault & f) {return f.type == type;});
}

std::map<int, double> EpsFaultInjection::unit_values(const std::string & type) const
{
  std::map<int, double> out;
  const auto * info = find_type(type);
  if (info == nullptr || *info->index == '\0') {
    return out;
  }
  for (const auto & f : active_) {
    if (f.type == type) {
      out[static_cast<int>(std::lround(param_or(f, info->index, 0.0)))] =
        param_or(f, info->value, info->default_value);
    }
  }
  return out;
}

std::set<int> EpsFaultInjection::indices(const std::string & type) const
{
  std::set<int> out;
  for (const auto & unit : unit_values(type)) {
    out.insert(unit.first);
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
