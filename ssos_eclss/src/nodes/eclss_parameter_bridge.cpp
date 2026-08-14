#include "ssos_eclss/nodes/eclss_parameter_bridge.hpp"

#include <algorithm>
#include <cmath>

namespace ssos_eclss
{
namespace nodes
{

namespace
{
// Helper: declare a double parameter only if not already declared.
void declare_double(rclcpp_lifecycle::LifecycleNode * n, const std::string & name,
                    double value)
{
  if (!n->has_parameter(name)) {
    n->declare_parameter(name, value);
  }
}
void declare_int(rclcpp_lifecycle::LifecycleNode * n, const std::string & name,
                 int value)
{
  if (!n->has_parameter(name)) {
    n->declare_parameter(name, value);
  }
}
}  // namespace

EclssParameterBridge::EclssParameterBridge(rclcpp_lifecycle::LifecycleNode * node)
: node_(node)
{}

void EclssParameterBridge::declare_ars_parameters(const ars::ArsParameters & d)
{
  // ---- Bed geometry (static) ----
  declare_double(node_, "ars.bed.desiccant.length", d.desiccant_bed.length);
  declare_double(node_, "ars.bed.desiccant.diameter", d.desiccant_bed.diameter);
  declare_double(node_, "ars.bed.desiccant.voidage", d.desiccant_bed.voidage);
  declare_double(node_, "ars.bed.desiccant.particle_diameter",
                 d.desiccant_bed.particle_diameter);
  declare_int(node_, "ars.bed.desiccant.n_cells",
              static_cast<int>(d.desiccant_bed.n_cells));
  declare_double(node_, "ars.bed.adsorbent.length", d.adsorbent_bed.length);
  declare_double(node_, "ars.bed.adsorbent.diameter", d.adsorbent_bed.diameter);
  declare_double(node_, "ars.bed.adsorbent.voidage", d.adsorbent_bed.voidage);
  declare_double(node_, "ars.bed.adsorbent.particle_diameter",
                 d.adsorbent_bed.particle_diameter);
  declare_int(node_, "ars.bed.adsorbent.n_cells",
              static_cast<int>(d.adsorbent_bed.n_cells));

  // ---- CO2-on-13X isotherm (dynamic, for what-if) ----
  declare_double(node_, "ars.isotherm.co2_13x.q_m0", d.co2_on_13x.q_m0);
  declare_double(node_, "ars.isotherm.co2_13x.b0", d.co2_on_13x.b0);
  declare_double(node_, "ars.isotherm.co2_13x.dH", d.co2_on_13x.dH);
  declare_double(node_, "ars.isotherm.co2_13x.t0", d.co2_on_13x.t0);

  // ---- LDF (dynamic) ----
  declare_double(node_, "ars.ldf.adsorbent.k_co2", d.adsorbent_ldf.k_co2);
  declare_double(node_, "ars.ldf.adsorbent.k_h2o", d.adsorbent_ldf.k_h2o);

  // ---- Heater (dynamic) ----
  declare_double(node_, "ars.heater.total_power_w", d.heater.total_power_w);
  declare_double(node_, "ars.heater.central_power_w", d.heater.central_power_w);
  declare_double(node_, "ars.heater.max_temp_k", d.heater.max_temp_k);

  // ---- Cycle timing (dynamic) ----
  declare_double(node_, "ars.cycle.air_save_s", d.cycle.air_save_s);
  declare_double(node_, "ars.cycle.adsorb_s", d.cycle.adsorb_s);
  declare_double(node_, "ars.cycle.vacuum_s", d.cycle.vacuum_s);

  // ---- Operating conditions (dynamic) ----
  declare_double(node_, "ars.operating.inlet_flow_scfm", d.operating.inlet_flow_scfm);
  declare_double(node_, "ars.operating.inlet_ppco2_torr",
                 d.operating.inlet_ppco2_torr);
  declare_double(node_, "ars.operating.ltl_inlet_temp_k", d.operating.ltl_inlet_temp_k);
  declare_double(node_, "ars.operating.ltl_flow_gpm", d.operating.ltl_flow_gpm);
  declare_double(node_, "ars.operating.cabin_temp_k", d.operating.cabin_temp_k);
  declare_double(node_, "ars.operating.cabin_pressure_pa",
                 d.operating.cabin_pressure_pa);
  declare_double(node_, "ars.operating.vacuum_pressure_pa",
                 d.operating.vacuum_pressure_pa);

  // ---- System efficiency (dynamic) ----
  declare_double(node_, "ars.efficiency.capture_efficiency",
                 d.efficiency.capture_efficiency);
  declare_double(node_, "ars.efficiency.holdup_loss", d.efficiency.holdup_loss);
}

ars::ArsParameters EclssParameterBridge::read_ars_parameters() const
{
  ars::ArsParameters p = ars::default_ars_parameters();
  auto gd = [&](const std::string & n, double def) {
    return node_->get_parameter_or(n, def);
  };

  p.desiccant_bed.length = gd("ars.bed.desiccant.length", p.desiccant_bed.length);
  p.desiccant_bed.diameter = gd("ars.bed.desiccant.diameter", p.desiccant_bed.diameter);
  p.desiccant_bed.voidage = gd("ars.bed.desiccant.voidage", p.desiccant_bed.voidage);
  p.desiccant_bed.particle_diameter =
    gd("ars.bed.desiccant.particle_diameter", p.desiccant_bed.particle_diameter);
  p.desiccant_bed.n_cells = static_cast<std::size_t>(
    node_->get_parameter_or("ars.bed.desiccant.n_cells",
                            static_cast<int>(p.desiccant_bed.n_cells)));
  p.adsorbent_bed.length = gd("ars.bed.adsorbent.length", p.adsorbent_bed.length);
  p.adsorbent_bed.diameter = gd("ars.bed.adsorbent.diameter", p.adsorbent_bed.diameter);
  p.adsorbent_bed.voidage = gd("ars.bed.adsorbent.voidage", p.adsorbent_bed.voidage);
  p.adsorbent_bed.particle_diameter =
    gd("ars.bed.adsorbent.particle_diameter", p.adsorbent_bed.particle_diameter);
  p.adsorbent_bed.n_cells = static_cast<std::size_t>(
    node_->get_parameter_or("ars.bed.adsorbent.n_cells",
                            static_cast<int>(p.adsorbent_bed.n_cells)));

  p.co2_on_13x.q_m0 = gd("ars.isotherm.co2_13x.q_m0", p.co2_on_13x.q_m0);
  p.co2_on_13x.b0 = gd("ars.isotherm.co2_13x.b0", p.co2_on_13x.b0);
  p.co2_on_13x.dH = gd("ars.isotherm.co2_13x.dH", p.co2_on_13x.dH);
  p.co2_on_13x.t0 = gd("ars.isotherm.co2_13x.t0", p.co2_on_13x.t0);

  p.adsorbent_ldf.k_co2 = gd("ars.ldf.adsorbent.k_co2", p.adsorbent_ldf.k_co2);
  p.adsorbent_ldf.k_h2o = gd("ars.ldf.adsorbent.k_h2o", p.adsorbent_ldf.k_h2o);

  p.heater.total_power_w = gd("ars.heater.total_power_w", p.heater.total_power_w);
  p.heater.central_power_w = gd("ars.heater.central_power_w", p.heater.central_power_w);
  p.heater.max_temp_k = gd("ars.heater.max_temp_k", p.heater.max_temp_k);

  p.cycle.air_save_s = gd("ars.cycle.air_save_s", p.cycle.air_save_s);
  p.cycle.adsorb_s = gd("ars.cycle.adsorb_s", p.cycle.adsorb_s);
  p.cycle.vacuum_s = gd("ars.cycle.vacuum_s", p.cycle.vacuum_s);

  p.operating.inlet_flow_scfm =
    gd("ars.operating.inlet_flow_scfm", p.operating.inlet_flow_scfm);
  p.operating.inlet_ppco2_torr =
    gd("ars.operating.inlet_ppco2_torr", p.operating.inlet_ppco2_torr);
  p.operating.ltl_inlet_temp_k =
    gd("ars.operating.ltl_inlet_temp_k", p.operating.ltl_inlet_temp_k);
  p.operating.ltl_flow_gpm = gd("ars.operating.ltl_flow_gpm", p.operating.ltl_flow_gpm);
  p.operating.cabin_temp_k = gd("ars.operating.cabin_temp_k", p.operating.cabin_temp_k);
  p.operating.cabin_pressure_pa =
    gd("ars.operating.cabin_pressure_pa", p.operating.cabin_pressure_pa);
  p.operating.vacuum_pressure_pa =
    gd("ars.operating.vacuum_pressure_pa", p.operating.vacuum_pressure_pa);

  p.efficiency.capture_efficiency =
    gd("ars.efficiency.capture_efficiency", p.efficiency.capture_efficiency);
  p.efficiency.holdup_loss = gd("ars.efficiency.holdup_loss", p.efficiency.holdup_loss);
  return p;
}

rcl_interfaces::msg::SetParametersResult EclssParameterBridge::validate(
  const std::vector<rclcpp::Parameter> & params) const
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  auto reject = [&](const std::string & msg) {
    result.successful = false;
    result.reason = msg;
  };

  for (const auto & p : params) {
    const std::string & name = p.get_name();

    // Toth heterogeneity exponent must lie in (0, 1].
    if (name == "ars.isotherm.co2_13x.t0") {
      const double t = p.as_double();
      if (!(t > 0.0 && t <= 1.0)) {
        reject("Toth exponent t0 must be in (0, 1], got " + std::to_string(t));
      }
    } else if (name == "ars.isotherm.co2_13x.q_m0" ||
               name == "ars.isotherm.co2_13x.b0" ||
               name == "ars.isotherm.co2_13x.dH") {
      if (p.as_double() <= 0.0) {
        reject(name + " must be positive, got " + std::to_string(p.as_double()));
      }
    } else if (name == "ars.bed.desiccant.voidage" ||
               name == "ars.bed.adsorbent.voidage") {
      const double v = p.as_double();
      if (!(v > 0.0 && v < 1.0)) {
        reject(name + " (voidage) must be in (0, 1), got " + std::to_string(v));
      }
    } else if (name == "ars.efficiency.capture_efficiency" ||
               name == "ars.efficiency.holdup_loss") {
      const double v = p.as_double();
      if (!(v >= 0.0 && v <= 1.0)) {
        reject(name + " must be in [0, 1], got " + std::to_string(v));
      }
    } else if (name == "ars.operating.cabin_temp_k" ||
               name == "ars.operating.ltl_inlet_temp_k" ||
               name == "ars.heater.max_temp_k") {
      if (p.as_double() <= 0.0) {
        reject(name + " (temperature) must be > 0 K, got " +
               std::to_string(p.as_double()));
      }
    } else if (name == "ars.operating.inlet_flow_scfm" ||
               name == "ars.operating.inlet_ppco2_torr" ||
               name == "ars.operating.ltl_flow_gpm" ||
               name == "ars.heater.total_power_w" ||
               name == "ars.heater.central_power_w" ||
               name == "ars.cycle.air_save_s" || name == "ars.cycle.adsorb_s" ||
               name == "ars.cycle.vacuum_s" || name == "ars.ldf.adsorbent.k_co2" ||
               name == "ars.ldf.adsorbent.k_h2o" ||
               name == "ars.bed.desiccant.length" ||
               name == "ars.bed.adsorbent.length" ||
               name == "ars.bed.desiccant.diameter" ||
               name == "ars.bed.adsorbent.diameter" ||
               name == "ars.bed.desiccant.particle_diameter" ||
               name == "ars.bed.adsorbent.particle_diameter") {
      if (p.as_double() < 0.0) {
        reject(name + " must be >= 0, got " + std::to_string(p.as_double()));
      }
    } else if (name == "ars.bed.desiccant.n_cells" ||
               name == "ars.bed.adsorbent.n_cells") {
      if (p.as_int() < 1) {
        reject(name + " must be >= 1, got " + std::to_string(p.as_int()));
      }
    }

    if (!result.successful) {
      break;
    }
  }
  return result;
}

bool EclssParameterBridge::is_static_parameter(const std::string & name)
{
  // Geometry and cell count require a reconfigure cycle.
  return name.find("ars.bed.") == 0;
}

}  // namespace nodes
}  // namespace ssos_eclss
