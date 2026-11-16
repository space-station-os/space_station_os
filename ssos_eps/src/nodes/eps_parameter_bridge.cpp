#include "ssos_eps/nodes/eps_parameter_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ssos_eps
{
namespace nodes
{

namespace
{

using system::EpsParameters;

constexpr double kInf = std::numeric_limits<double>::infinity();

struct DoubleField
{
  const char * name;
  double & (*ref)(EpsParameters &);
  double min;
  double max;
  bool min_exclusive;
};

struct IntField
{
  const char * name;
  int & (*ref)(EpsParameters &);
  int min;
};

struct BoolField
{
  const char * name;
  bool & (*ref)(EpsParameters &);
};

struct IntArrayField
{
  const char * name;
  std::vector<int> & (*ref)(EpsParameters &);
};

struct ArrayField
{
  const char * name;
  std::vector<double> & (*ref)(EpsParameters &);
};

const std::vector<DoubleField> & double_fields()
{
  static const std::vector<DoubleField> fields = {
    {"eps.low_soc_threshold",
      [](EpsParameters & p) -> double & {return p.low_soc_threshold;}, 0.0, 1.0, false},

    {"solar.array.area_m2",
      [](EpsParameters & p) -> double & {return p.solar.array.area_m2;}, 0.0, kInf, false},
    {"solar.array.cell_efficiency",
      [](EpsParameters & p) -> double & {return p.solar.array.cell_efficiency;}, 0.0, 1.0, true},
    {"solar.array.degradation_factor",
      [](EpsParameters & p) -> double & {return p.solar.array.degradation_factor;},
      0.0, 1.0, false},
    {"solar.array.sunrise_ramp_s",
      [](EpsParameters & p) -> double & {return p.solar.array.sunrise_ramp_s;}, 0.0, kInf, false},
    {"solar.array.beta_max_deg",
      [](EpsParameters & p) -> double & {return p.solar.array.beta_max_deg;}, 0.0, 90.0, false},
    {"solar.array.beta_lock_limit_deg",
      [](EpsParameters & p) -> double & {return p.solar.array.beta_lock_limit_deg;},
      0.0, 90.0, false},
    {"solar.array.gimbal_lock_derate",
      [](EpsParameters & p) -> double & {return p.solar.array.gimbal_lock_derate;},
      0.0, 1.0, false},
    {"solar.array.fixed_beta_deg",
      [](EpsParameters & p) -> double & {return p.solar.array.fixed_beta_deg;},
      -90.0, 90.0, false},

    {"solar.thermal.ambient_temp_c",
      [](EpsParameters & p) -> double & {return p.solar.thermal.ambient_temp_c;},
      -150.0, 150.0, false},
    {"solar.thermal.temp_rise_per_kw_c",
      [](EpsParameters & p) -> double & {return p.solar.thermal.temp_rise_per_kw_c;},
      0.0, kInf, false},
    {"solar.thermal.time_constant_s",
      [](EpsParameters & p) -> double & {return p.solar.thermal.time_constant_s;},
      0.0, kInf, false},

    {"solar.ssu.voltage_setpoint_v",
      [](EpsParameters & p) -> double & {return p.solar.ssu.voltage_setpoint_v;},
      0.0, kInf, true},
    {"solar.ssu.voltage_ripple_v",
      [](EpsParameters & p) -> double & {return p.solar.ssu.voltage_ripple_v;}, 0.0, kInf, false},
    {"solar.ssu.ripple_period_s",
      [](EpsParameters & p) -> double & {return p.solar.ssu.ripple_period_s;}, 0.0, kInf, false},
    {"solar.ssu.efficiency",
      [](EpsParameters & p) -> double & {return p.solar.ssu.efficiency;}, 0.0, 1.0, true},
    {"solar.ssu.min_input_power_w",
      [](EpsParameters & p) -> double & {return p.solar.ssu.min_input_power_w;},
      0.0, kInf, false},

    {"battery.capacity_ah",
      [](EpsParameters & p) -> double & {return p.battery.capacity_ah;}, 0.0, kInf, true},
    {"battery.state_of_health",
      [](EpsParameters & p) -> double & {return p.battery.state_of_health;}, 0.0, 1.0, true},
    {"battery.internal_resistance_ohm",
      [](EpsParameters & p) -> double & {return p.battery.internal_resistance_ohm;},
      0.0, kInf, false},
    {"battery.coulombic_efficiency",
      [](EpsParameters & p) -> double & {return p.battery.coulombic_efficiency;},
      0.0, 1.0, true},
    {"battery.max_charge_current_a",
      [](EpsParameters & p) -> double & {return p.battery.max_charge_current_a;},
      0.0, kInf, false},
    {"battery.max_discharge_current_a",
      [](EpsParameters & p) -> double & {return p.battery.max_discharge_current_a;},
      0.0, kInf, false},
    {"battery.max_cell_voltage_v",
      [](EpsParameters & p) -> double & {return p.battery.max_cell_voltage_v;},
      0.0, kInf, true},
    {"battery.min_cell_voltage_v",
      [](EpsParameters & p) -> double & {return p.battery.min_cell_voltage_v;},
      0.0, kInf, true},
    {"battery.thermal_mass_j_k",
      [](EpsParameters & p) -> double & {return p.battery.thermal_mass_j_k;}, 0.0, kInf, true},
    {"battery.heat_transfer_w_k",
      [](EpsParameters & p) -> double & {return p.battery.heat_transfer_w_k;},
      0.0, kInf, false},
    {"battery.coldplate_temp_c",
      [](EpsParameters & p) -> double & {return p.battery.coldplate_temp_c;},
      -100.0, 100.0, false},
    {"battery.max_temp_c",
      [](EpsParameters & p) -> double & {return p.battery.max_temp_c;}, -100.0, 200.0, false},
    {"battery.initial_soc",
      [](EpsParameters & p) -> double & {return p.battery.initial_soc;}, 0.0, 1.0, false},
    {"battery.initial_temp_c",
      [](EpsParameters & p) -> double & {return p.battery.initial_temp_c;},
      -100.0, 100.0, false},

    {"bcdu.charge_enter_v",
      [](EpsParameters & p) -> double & {return p.bcdu.charge_enter_v;}, 0.0, kInf, true},
    {"bcdu.discharge_enter_v",
      [](EpsParameters & p) -> double & {return p.bcdu.discharge_enter_v;}, 0.0, kInf, false},
    {"bcdu.regulation_voltage_v",
      [](EpsParameters & p) -> double & {return p.bcdu.regulation_voltage_v;}, 0.0, kInf, true},
    {"bcdu.regulation_min_v",
      [](EpsParameters & p) -> double & {return p.bcdu.regulation_min_v;}, 0.0, kInf, true},
    {"bcdu.regulation_max_v",
      [](EpsParameters & p) -> double & {return p.bcdu.regulation_max_v;}, 0.0, kInf, true},
    {"bcdu.max_charge_current_a",
      [](EpsParameters & p) -> double & {return p.bcdu.max_charge_current_a;},
      0.0, kInf, false},
    {"bcdu.max_discharge_current_a",
      [](EpsParameters & p) -> double & {return p.bcdu.max_discharge_current_a;},
      0.0, kInf, false},
    {"bcdu.charge_efficiency",
      [](EpsParameters & p) -> double & {return p.bcdu.charge_efficiency;}, 0.0, 1.0, true},
    {"bcdu.discharge_efficiency",
      [](EpsParameters & p) -> double & {return p.bcdu.discharge_efficiency;}, 0.0, 1.0, true},
    {"bcdu.deadband_w",
      [](EpsParameters & p) -> double & {return p.bcdu.deadband_w;}, 0.0, kInf, false},
    {"bcdu.unserved_tolerance_w",
      [](EpsParameters & p) -> double & {return p.bcdu.unserved_tolerance_w;},
      0.0, kInf, false},

    {"mbsu.min_source_voltage_v",
      [](EpsParameters & p) -> double & {return p.mbsu.min_source_voltage_v;},
      0.0, kInf, true},
    {"mbsu.max_source_voltage_v",
      [](EpsParameters & p) -> double & {return p.mbsu.max_source_voltage_v;},
      0.0, kInf, true},

    {"ddcu.input_min_v",
      [](EpsParameters & p) -> double & {return p.ddcu.input_min_v;}, 0.0, kInf, true},
    {"ddcu.input_max_v",
      [](EpsParameters & p) -> double & {return p.ddcu.input_max_v;}, 0.0, kInf, true},
    {"ddcu.output_voltage_v",
      [](EpsParameters & p) -> double & {return p.ddcu.output_voltage_v;}, 0.0, kInf, true},
    {"ddcu.regulation_droop_v",
      [](EpsParameters & p) -> double & {return p.ddcu.regulation_droop_v;}, 0.0, kInf, false},
    {"ddcu.rated_power_w",
      [](EpsParameters & p) -> double & {return p.ddcu.rated_power_w;}, 0.0, kInf, true},
    {"ddcu.efficiency",
      [](EpsParameters & p) -> double & {return p.ddcu.efficiency;}, 0.0, 1.0, true},
    {"ddcu.no_load_loss_w",
      [](EpsParameters & p) -> double & {return p.ddcu.no_load_loss_w;}, 0.0, kInf, false},
    {"ddcu.thermal_mass_j_k",
      [](EpsParameters & p) -> double & {return p.ddcu.thermal_mass_j_k;}, 0.0, kInf, true},
    {"ddcu.heat_transfer_w_k",
      [](EpsParameters & p) -> double & {return p.ddcu.heat_transfer_w_k;}, 0.0, kInf, false},
    {"ddcu.coldplate_temp_c",
      [](EpsParameters & p) -> double & {return p.ddcu.coldplate_temp_c;},
      -100.0, 100.0, false},
    {"ddcu.trip_temp_c",
      [](EpsParameters & p) -> double & {return p.ddcu.trip_temp_c;}, -100.0, 200.0, false},
    {"ddcu.reset_temp_c",
      [](EpsParameters & p) -> double & {return p.ddcu.reset_temp_c;}, -100.0, 200.0, false},
    {"ddcu.initial_temp_c",
      [](EpsParameters & p) -> double & {return p.ddcu.initial_temp_c;},
      -100.0, 100.0, false},

    {"loads.demand_scale",
      [](EpsParameters & p) -> double & {return p.demand_scale;}, 0.0, 10.0, false},
  };
  return fields;
}

const std::vector<IntField> & int_fields()
{
  static const std::vector<IntField> fields = {
    {"eps.n_channels", [](EpsParameters & p) -> int & {return p.n_channels;}, 1},
    {"eps.orus_per_channel", [](EpsParameters & p) -> int & {return p.orus_per_channel;}, 1},
    {"battery.n_series_cells",
      [](EpsParameters & p) -> int & {return p.battery.n_series_cells;}, 1},
    {"solar.array.beta_precession_orbits",
      [](EpsParameters & p) -> int & {return p.solar.array.beta_precession_orbits;}, 0},
    {"eps.n_mbsu", [](EpsParameters & p) -> int & {return p.n_mbsu;}, 1},
    {"mbsu.min_sources", [](EpsParameters & p) -> int & {return p.mbsu.min_sources;}, 1},
    {"mbsu.redundant_sources",
      [](EpsParameters & p) -> int & {return p.mbsu.redundant_sources;}, 1},
  };
  return fields;
}

const std::vector<BoolField> & bool_fields()
{
  static const std::vector<BoolField> fields = {
    {"solar.array.derate_with_beta_cosine",
      [](EpsParameters & p) -> bool & {return p.solar.array.derate_with_beta_cosine;}},
    {"solar.array.use_fixed_beta",
      [](EpsParameters & p) -> bool & {return p.solar.array.use_fixed_beta;}},
  };
  return fields;
}

const std::vector<ArrayField> & array_fields()
{
  static const std::vector<ArrayField> fields = {
    {"battery.soc_points",
      [](EpsParameters & p) -> std::vector<double> & {return p.battery.soc_points;}},
    {"battery.cell_ocv_v",
      [](EpsParameters & p) -> std::vector<double> & {return p.battery.cell_ocv_v;}},
  };
  return fields;
}

const std::vector<IntArrayField> & int_array_fields()
{
  static const std::vector<IntArrayField> fields = {
    {"eps.channel_mbsu",
      [](EpsParameters & p) -> std::vector<int> & {return p.channel_mbsu;}},
    {"eps.ddcu_mbsu",
      [](EpsParameters & p) -> std::vector<int> & {return p.ddcu_mbsu;}},
  };
  return fields;
}

// Baseline loads are four parallel arrays, one entry per load.
const char * const kLoadIds = "loads.ids";
const char * const kLoadPower = "loads.power_w";
const char * const kLoadPriority = "loads.priority";
const char * const kLoadDdcu = "loads.ddcu";

const std::vector<std::string> & static_parameters()
{
  static const std::vector<std::string> names = {
    "eps.n_channels", "eps.orus_per_channel", "battery.n_series_cells",
    "battery.capacity_ah", "battery.soc_points", "battery.cell_ocv_v",
    "battery.initial_soc", "battery.initial_temp_c",
    "eps.n_mbsu", "eps.channel_mbsu", "eps.ddcu_mbsu",
    kLoadIds, kLoadPower, kLoadPriority, kLoadDdcu,
  };
  return names;
}

std::vector<int64_t> to_int64(const std::vector<int> & v)
{
  return std::vector<int64_t>(v.begin(), v.end());
}

std::vector<int> to_int(const std::vector<int64_t> & v)
{
  std::vector<int> out;
  out.reserve(v.size());
  for (auto x : v) {
    out.push_back(static_cast<int>(x));
  }
  return out;
}

const rclcpp::Parameter * find(const std::vector<rclcpp::Parameter> & params,
                               const std::string & name)
{
  for (const auto & p : params) {
    if (p.get_name() == name) {
      return &p;
    }
  }
  return nullptr;
}

std::string fmt(double v)
{
  return std::to_string(v);
}

}  // namespace

EpsParameterBridge::EpsParameterBridge(rclcpp_lifecycle::LifecycleNode * node)
: node_(node)
{}

void EpsParameterBridge::declare_parameters(const system::EpsParameters & defaults)
{
  EpsParameters d = defaults;
  for (const auto & f : double_fields()) {
    if (!node_->has_parameter(f.name)) {
      node_->declare_parameter(f.name, f.ref(d));
    }
  }
  for (const auto & f : int_fields()) {
    if (!node_->has_parameter(f.name)) {
      node_->declare_parameter(f.name, f.ref(d));
    }
  }
  for (const auto & f : bool_fields()) {
    if (!node_->has_parameter(f.name)) {
      node_->declare_parameter(f.name, f.ref(d));
    }
  }
  for (const auto & f : array_fields()) {
    if (!node_->has_parameter(f.name)) {
      node_->declare_parameter(f.name, f.ref(d));
    }
  }
  for (const auto & f : int_array_fields()) {
    if (!node_->has_parameter(f.name)) {
      node_->declare_parameter(f.name, to_int64(f.ref(d)));
    }
  }

  std::vector<std::string> ids;
  std::vector<double> power;
  std::vector<int64_t> priority;
  std::vector<int64_t> ddcu;
  for (const auto & load : d.baseline_loads) {
    ids.push_back(load.id);
    power.push_back(load.power_w);
    priority.push_back(load.priority);
    ddcu.push_back(load.ddcu);
  }
  if (!node_->has_parameter(kLoadIds)) {
    node_->declare_parameter(kLoadIds, ids);
  }
  if (!node_->has_parameter(kLoadPower)) {
    node_->declare_parameter(kLoadPower, power);
  }
  if (!node_->has_parameter(kLoadPriority)) {
    node_->declare_parameter(kLoadPriority, priority);
  }
  if (!node_->has_parameter(kLoadDdcu)) {
    node_->declare_parameter(kLoadDdcu, ddcu);
  }
}

system::EpsParameters EpsParameterBridge::read_parameters() const
{
  EpsParameters p = system::default_eps_parameters();
  for (const auto & f : double_fields()) {
    f.ref(p) = node_->get_parameter_or(f.name, f.ref(p));
  }
  for (const auto & f : int_fields()) {
    f.ref(p) = static_cast<int>(node_->get_parameter_or(
        f.name, static_cast<int64_t>(f.ref(p))));
  }
  for (const auto & f : bool_fields()) {
    f.ref(p) = node_->get_parameter_or(f.name, f.ref(p));
  }
  for (const auto & f : array_fields()) {
    f.ref(p) = node_->get_parameter_or(f.name, f.ref(p));
  }
  for (const auto & f : int_array_fields()) {
    f.ref(p) = to_int(node_->get_parameter_or(f.name, to_int64(f.ref(p))));
  }

  const auto ids = node_->get_parameter_or(kLoadIds, std::vector<std::string>{});
  const auto power = node_->get_parameter_or(kLoadPower, std::vector<double>{});
  const auto priority = node_->get_parameter_or(kLoadPriority, std::vector<int64_t>{});
  const auto ddcu = node_->get_parameter_or(kLoadDdcu, std::vector<int64_t>{});
  p.baseline_loads.clear();
  for (std::size_t k = 0; k < ids.size(); ++k) {
    loads::LoadSpec spec;
    spec.id = ids[k];
    spec.power_w = k < power.size() ? power[k] : 0.0;
    spec.priority = k < priority.size() ? static_cast<int>(priority[k]) : 0;
    spec.ddcu = k < ddcu.size() ? static_cast<int>(ddcu[k]) : -1;
    p.baseline_loads.push_back(spec);
  }
  return p;
}

bool EpsParameterBridge::is_physics_parameter(const std::string & name)
{
  for (const auto & f : double_fields()) {
    if (name == f.name) {
      return true;
    }
  }
  for (const auto & f : int_fields()) {
    if (name == f.name) {
      return true;
    }
  }
  for (const auto & f : bool_fields()) {
    if (name == f.name) {
      return true;
    }
  }
  for (const auto & f : array_fields()) {
    if (name == f.name) {
      return true;
    }
  }
  for (const auto & f : int_array_fields()) {
    if (name == f.name) {
      return true;
    }
  }
  return name == kLoadIds || name == kLoadPower || name == kLoadPriority ||
         name == kLoadDdcu;
}

bool EpsParameterBridge::is_static_parameter(const std::string & name)
{
  const auto & names = static_parameters();
  return std::find(names.begin(), names.end(), name) != names.end();
}

double EpsParameterBridge::proposed_double(
  const std::string & name, const std::vector<rclcpp::Parameter> & params) const
{
  if (const auto * p = find(params, name)) {
    return p->as_double();
  }
  return node_->get_parameter(name).as_double();
}

std::vector<double> EpsParameterBridge::proposed_array(
  const std::string & name, const std::vector<rclcpp::Parameter> & params) const
{
  if (const auto * p = find(params, name)) {
    return p->as_double_array();
  }
  return node_->get_parameter(name).as_double_array();
}

int64_t EpsParameterBridge::proposed_int(
  const std::string & name, const std::vector<rclcpp::Parameter> & params) const
{
  if (const auto * p = find(params, name)) {
    return p->as_int();
  }
  return node_->get_parameter(name).as_int();
}

std::vector<int64_t> EpsParameterBridge::proposed_int_array(
  const std::string & name, const std::vector<rclcpp::Parameter> & params) const
{
  if (const auto * p = find(params, name)) {
    return p->as_integer_array();
  }
  return node_->get_parameter(name).as_integer_array();
}

std::vector<std::string> EpsParameterBridge::proposed_string_array(
  const std::string & name, const std::vector<rclcpp::Parameter> & params) const
{
  if (const auto * p = find(params, name)) {
    return p->as_string_array();
  }
  return node_->get_parameter(name).as_string_array();
}

rcl_interfaces::msg::SetParametersResult EpsParameterBridge::validate(
  const std::vector<rclcpp::Parameter> & params) const
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  auto reject = [&](const std::string & msg) {
    result.successful = false;
    result.reason = msg;
  };

  bool touches_cell_limits = false;
  bool touches_hysteresis = false;
  bool touches_regulation = false;
  bool touches_ocv_table = false;
  bool touches_source_window = false;
  bool touches_ddcu_window = false;
  bool touches_ddcu_trip = false;
  bool touches_topology = false;
  bool touches_loads = false;

  for (const auto & p : params) {
    const std::string & name = p.get_name();

    for (const auto & f : double_fields()) {
      if (name != f.name) {
        continue;
      }
      const double v = p.as_double();
      const bool below = f.min_exclusive ? !(v > f.min) : !(v >= f.min);
      if (!std::isfinite(v) || below || v > f.max) {
        reject(name + " must be in " + (f.min_exclusive ? "(" : "[") + fmt(f.min) + ", " +
               (std::isinf(f.max) ? std::string("inf)") : fmt(f.max) + "]") +
               ", got " + fmt(v));
      }
    }
    for (const auto & f : int_fields()) {
      if (name == f.name && p.as_int() < f.min) {
        reject(name + " must be >= " + std::to_string(f.min) + ", got " +
               std::to_string(p.as_int()));
      }
    }

    touches_cell_limits = touches_cell_limits || name == "battery.max_cell_voltage_v" ||
      name == "battery.min_cell_voltage_v";
    touches_hysteresis = touches_hysteresis || name == "bcdu.charge_enter_v" ||
      name == "bcdu.discharge_enter_v";
    touches_regulation = touches_regulation || name == "bcdu.regulation_voltage_v" ||
      name == "bcdu.regulation_min_v" || name == "bcdu.regulation_max_v";
    touches_ocv_table = touches_ocv_table || name == "battery.soc_points" ||
      name == "battery.cell_ocv_v";
    touches_source_window = touches_source_window || name == "mbsu.min_source_voltage_v" ||
      name == "mbsu.max_source_voltage_v";
    touches_ddcu_window = touches_ddcu_window || name == "ddcu.input_min_v" ||
      name == "ddcu.input_max_v";
    touches_ddcu_trip = touches_ddcu_trip || name == "ddcu.trip_temp_c" ||
      name == "ddcu.reset_temp_c";
    touches_topology = touches_topology || name == "eps.n_channels" || name == "eps.n_mbsu" ||
      name == "eps.channel_mbsu" || name == "eps.ddcu_mbsu";
    touches_loads = touches_loads || name == kLoadIds || name == kLoadPower ||
      name == kLoadPriority || name == kLoadDdcu || name == "eps.ddcu_mbsu";
  }
  if (!result.successful) {
    return result;
  }

  if (touches_cell_limits &&
    proposed_double("battery.min_cell_voltage_v", params) >=
    proposed_double("battery.max_cell_voltage_v", params))
  {
    reject("battery.min_cell_voltage_v must be below battery.max_cell_voltage_v");
  }
  if (touches_hysteresis &&
    proposed_double("bcdu.discharge_enter_v", params) >=
    proposed_double("bcdu.charge_enter_v", params))
  {
    reject("bcdu.discharge_enter_v must be below bcdu.charge_enter_v");
  }
  if (touches_regulation) {
    const double lo = proposed_double("bcdu.regulation_min_v", params);
    const double hi = proposed_double("bcdu.regulation_max_v", params);
    const double v = proposed_double("bcdu.regulation_voltage_v", params);
    if (!(lo <= v && v <= hi)) {
      reject("bcdu.regulation_voltage_v must lie within [regulation_min_v, regulation_max_v]");
    }
  }
  if (touches_ocv_table) {
    const auto soc = proposed_array("battery.soc_points", params);
    const auto ocv = proposed_array("battery.cell_ocv_v", params);
    if (soc.size() < 2 || soc.size() != ocv.size()) {
      reject("battery.soc_points and battery.cell_ocv_v need the same length (>= 2)");
    } else {
      for (std::size_t i = 0; i < soc.size(); ++i) {
        if (soc[i] < 0.0 || soc[i] > 1.0 || (i > 0 && soc[i] <= soc[i - 1])) {
          reject("battery.soc_points must be strictly increasing within [0, 1]");
          break;
        }
        if (ocv[i] <= 0.0 || (i > 0 && ocv[i] < ocv[i - 1])) {
          reject("battery.cell_ocv_v must be positive and non-decreasing");
          break;
        }
      }
    }
  }
  if (touches_source_window &&
    proposed_double("mbsu.min_source_voltage_v", params) >=
    proposed_double("mbsu.max_source_voltage_v", params))
  {
    reject("mbsu.min_source_voltage_v must be below mbsu.max_source_voltage_v");
  }
  if (touches_ddcu_window &&
    proposed_double("ddcu.input_min_v", params) >= proposed_double("ddcu.input_max_v", params))
  {
    reject("ddcu.input_min_v must be below ddcu.input_max_v");
  }
  if (touches_ddcu_trip &&
    proposed_double("ddcu.reset_temp_c", params) >= proposed_double("ddcu.trip_temp_c", params))
  {
    reject("ddcu.reset_temp_c must be below ddcu.trip_temp_c");
  }
  if (touches_topology) {
    const auto n_channels = proposed_int("eps.n_channels", params);
    const auto n_mbsu = proposed_int("eps.n_mbsu", params);
    const auto channel_mbsu = proposed_int_array("eps.channel_mbsu", params);
    const auto ddcu_mbsu = proposed_int_array("eps.ddcu_mbsu", params);
    if (static_cast<int64_t>(channel_mbsu.size()) != n_channels) {
      reject("eps.channel_mbsu needs one entry per channel (" + std::to_string(n_channels) + ")");
    } else if (ddcu_mbsu.empty()) {
      reject("eps.ddcu_mbsu needs at least one DDCU");
    } else {
      for (auto m : channel_mbsu) {
        if (m < -1 || m >= n_mbsu) {
          reject("eps.channel_mbsu entries must be -1 or an MBSU index below eps.n_mbsu");
          break;
        }
      }
      for (auto m : ddcu_mbsu) {
        if (m < -1 || m >= n_mbsu) {
          reject("eps.ddcu_mbsu entries must be -1 or an MBSU index below eps.n_mbsu");
          break;
        }
      }
    }
  }
  if (touches_loads) {
    const auto ids = proposed_string_array(kLoadIds, params);
    const auto power = proposed_array(kLoadPower, params);
    const auto priority = proposed_int_array(kLoadPriority, params);
    const auto ddcu = proposed_int_array(kLoadDdcu, params);
    const auto n_ddcu = static_cast<int64_t>(proposed_int_array("eps.ddcu_mbsu", params).size());
    if (power.size() != ids.size() || priority.size() != ids.size() ||
      ddcu.size() != ids.size())
    {
      reject("loads.ids, loads.power_w, loads.priority and loads.ddcu need the same length");
    } else {
      for (std::size_t k = 0; k < ids.size(); ++k) {
        if (ids[k].empty() ||
          std::count(ids.begin(), ids.end(), ids[k]) != 1)
        {
          reject("loads.ids must be unique and non-empty");
          break;
        }
        if (!(power[k] > 0.0) || !std::isfinite(power[k])) {
          reject("loads.power_w entries must be positive");
          break;
        }
        if (priority[k] < 0) {
          reject("loads.priority entries must be >= 0");
          break;
        }
        if (ddcu[k] < -1 || ddcu[k] >= n_ddcu) {
          reject("loads.ddcu entries must be -1 or a DDCU index");
          break;
        }
      }
    }
  }
  return result;
}

}  // namespace nodes
}  // namespace ssos_eps
