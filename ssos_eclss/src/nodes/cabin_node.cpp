#include "ssos_eclss/nodes/cabin_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include "ssos_eclss/common/units.hpp"
#include "ssos_eclss/nodes/eclss_diagnostics.hpp"

using std::placeholders::_1;

namespace ssos_eclss
{
namespace nodes
{

namespace
{
constexpr double kMaxSubstepS = 60.0;    // explicit gas balance stays stable
}

CabinNode::CabinNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("cabin_node", options),
  drills_(this, "cabin"),
  health_(this, "cabin",
    {"/ssos/cabin/atmosphere", "/ssos/cabin/co2_ppm", "/ssos/cabin/condensate_kg_day",
      "/ssos/cabin/diagnostics"},
    {"/ssos/crew/co2_kg_day", "/ssos/crew/o2_consumption_kg_day",
      "/ssos/crew/latent_water_kg_day", "/ssos/ars/co2_removal_kg_day", "/ssos/ogs/o2_kg_day"})
{
  this->declare_parameter("step_rate_hz", step_rate_hz_);
  this->declare_parameter("cabin_volume_m3", cabin_volume_m3_);
  this->declare_parameter("cabin_temp_c", cabin_temp_c_);
  this->declare_parameter("thermal_node", thermal_node_);
  this->declare_parameter("initial_co2_ppm", initial_co2_ppm_);
  this->declare_parameter("initial_relative_humidity", initial_rh_);
  this->declare_parameter("leak_area_m2", leak_area_m2_);
  this->declare_parameter("co2_alarm_ppm", co2_alarm_ppm_);
  this->declare_parameter("o2_low_pct", o2_low_pct_);
  this->declare_parameter("o2_high_pct", o2_high_pct_);
  this->declare_parameter("pressure_low_kpa", pressure_low_kpa_);
  this->declare_parameter("pressure_high_kpa", pressure_high_kpa_);
  this->declare_parameter("rh_low", rh_low_);
  this->declare_parameter("rh_high", rh_high_);
  this->declare_parameter("leak_alarm_kg_day", leak_alarm_kg_day_);
  this->declare_parameter("o2_sensor_tolerance_pct", o2_sensor_tolerance_pct_);
  this->declare_parameter("enable_auto_faults", enable_auto_faults_);
  autostart_timer_ = EclssDiagnostics::maybe_autostart(this);
}

CallbackReturn CabinNode::on_configure(const rclcpp_lifecycle::State &)
{
  step_rate_hz_ = this->get_parameter("step_rate_hz").as_double();
  cabin_volume_m3_ = this->get_parameter("cabin_volume_m3").as_double();
  cabin_temp_c_ = this->get_parameter("cabin_temp_c").as_double();
  thermal_node_ = this->get_parameter("thermal_node").as_string();
  initial_co2_ppm_ = this->get_parameter("initial_co2_ppm").as_double();
  initial_rh_ = this->get_parameter("initial_relative_humidity").as_double();
  leak_area_m2_ = this->get_parameter("leak_area_m2").as_double();
  co2_alarm_ppm_ = this->get_parameter("co2_alarm_ppm").as_double();
  o2_low_pct_ = this->get_parameter("o2_low_pct").as_double();
  o2_high_pct_ = this->get_parameter("o2_high_pct").as_double();
  pressure_low_kpa_ = this->get_parameter("pressure_low_kpa").as_double();
  pressure_high_kpa_ = this->get_parameter("pressure_high_kpa").as_double();
  rh_low_ = this->get_parameter("rh_low").as_double();
  rh_high_ = this->get_parameter("rh_high").as_double();
  leak_alarm_kg_day_ = this->get_parameter("leak_alarm_kg_day").as_double();
  o2_sensor_tolerance_pct_ = this->get_parameter("o2_sensor_tolerance_pct").as_double();
  enable_auto_faults_ = this->get_parameter("enable_auto_faults").as_bool();

  cabin::CabinParams cp{};
  cp.volume_m3 = cabin_volume_m3_;
  cp.temperature_k = units::celsius_to_kelvin(cabin_temp_c_);
  atmosphere_ = std::make_unique<cabin::CabinAtmosphere>(cp);
  atmosphere_->initialize_nominal(initial_co2_ppm_, initial_rh_);
  cabin::LeakParams lp{};
  lp.nominal_area_m2 = leak_area_m2_;
  lp.discharge_coeff = 0.62;
  leak_ = std::make_unique<cabin::LeakModel>(lp);
  control_.reset();
  last_control_ = {};
  leak_kg_s_ = 0.0;

  atmosphere_pub_ = this->create_publisher<CabinAtmosphereMsg>("/ssos/cabin/atmosphere", 10);
  co2_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ssos/cabin/co2_ppm", 10);
  condensate_pub_ =
    this->create_publisher<std_msgs::msg::Float64>("/ssos/cabin/condensate_kg_day", 10);
  telemetry_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/ssos/cabin/diagnostics", 10);
  health_.configure();
  drills_.start();

  crew_co2_.subscribe(this, "/ssos/crew/co2_kg_day");
  crew_o2_.subscribe(this, "/ssos/crew/o2_consumption_kg_day");
  crew_latent_.subscribe(this, "/ssos/crew/latent_water_kg_day");
  ars_removal_.subscribe(this, "/ssos/ars/co2_removal_kg_day");
  ogs_o2_.subscribe(this, "/ssos/ogs/o2_kg_day");
  // The thermal network's cabin node sets the air temperature when it runs.
  thermal_sub_ = this->create_subscription<space_station_interfaces::msg::ThermalNodeDataArray>(
    "/thermal/nodes/state", 10,
    [this](const space_station_interfaces::msg::ThermalNodeDataArray::SharedPtr msg) {
      for (const auto & n : msg->nodes) {
        if (n.name == thermal_node_) {
          atmosphere_->set_temperature(units::celsius_to_kelvin(n.temperature));
          return;
        }
      }
    });
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CabinNode::on_set_parameters, this, _1));
  return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult CabinNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : params) {
    const std::string & n = p.get_name();
    if (n == "co2_alarm_ppm") {
      co2_alarm_ppm_ = p.as_double();
    } else if (n == "step_rate_hz") {
      step_rate_hz_ = p.as_double();
    } else if (n == "enable_auto_faults") {
      enable_auto_faults_ = p.as_bool();
    } else if (n == "leak_alarm_kg_day") {
      leak_alarm_kg_day_ = p.as_double();
    }
  }
  return result;
}

CallbackReturn CabinNode::on_activate(const rclcpp_lifecycle::State &)
{
  atmosphere_pub_->on_activate();
  co2_pub_->on_activate();
  condensate_pub_->on_activate();
  telemetry_pub_->on_activate();
  health_.activate();
  clock_.reset();
  const auto period = std::chrono::duration<double>(1.0 / std::max(step_rate_hz_, 1.0e-3));
  step_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&CabinNode::step, this));
  return CallbackReturn::SUCCESS;
}

CallbackReturn CabinNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  atmosphere_pub_->on_deactivate();
  co2_pub_->on_deactivate();
  condensate_pub_->on_deactivate();
  telemetry_pub_->on_deactivate();
  health_.deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CabinNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  for (auto * in : {&crew_co2_, &crew_o2_, &crew_latent_, &ars_removal_, &ogs_o2_}) {
    in->reset();
  }
  thermal_sub_.reset();
  atmosphere_pub_.reset();
  co2_pub_.reset();
  condensate_pub_.reset();
  telemetry_pub_.reset();
  health_.cleanup();
  drills_.stop();
  atmosphere_.reset();
  leak_.reset();
  return CallbackReturn::SUCCESS;
}

void CabinNode::apply_drill()
{
  leak_->set_fault_area(drills_.active("cabin_leak") ? drills_.param("area_m2", 3.0e-6) : 0.0);
  control_.set_chx_failed(drills_.active("chx_failure"));
}

void CabinNode::step()
{
  const rclcpp::Time now = this->now();
  if (drills_.update(now)) {
    apply_drill();
  }
  const double dt = clock_.advance(now);
  if (dt > 0.0) {
    // Held over the step: crew, ARS and OGS rates from their nodes.
    const double co2_mol_s = (crew_co2_.get() - ars_removal_.get()) / units::M_CO2;
    const double o2_mol_s = (ogs_o2_.get() - crew_o2_.get()) / units::M_O2;
    const double h2o_mol_s = crew_latent_.get() / units::M_H2O;

    const int n = std::max(1, static_cast<int>(std::ceil(dt / kMaxSubstepS)));
    const double h = dt / n;
    double condensate_kg = 0.0, n2_kg = 0.0, o2_kg = 0.0, leak_kg = 0.0;
    for (int i = 0; i < n; ++i) {
      const cabin::GasFlows lk = leak_->leak_flows(*atmosphere_);
      leak_kg += leak_->mass_leak_rate(*atmosphere_) * h;
      const cabin::ControlResult c = control_.step(h, *atmosphere_);
      condensate_kg += c.condensate_kg_s * h;
      n2_kg += c.n2_makeup_kg_s * h;
      o2_kg += c.o2_makeup_kg_s * h;
      cabin::GasFlows total{};
      total.co2 = co2_mol_s + lk.co2 + c.flows.co2;
      total.o2 = o2_mol_s + lk.o2 + c.flows.o2;
      total.n2 = lk.n2 + c.flows.n2;
      total.h2o = h2o_mol_s + lk.h2o + c.flows.h2o;
      atmosphere_->apply_flows(h, total);
    }
    last_control_.condensate_kg_s = condensate_kg / dt;
    last_control_.n2_makeup_kg_s = n2_kg / dt;
    last_control_.o2_makeup_kg_s = o2_kg / dt;
    leak_kg_s_ = leak_kg / dt;
  }
  publish(now);
  check_health(now);
}

void CabinNode::publish(const rclcpp::Time & now)
{
  const auto & atm = *atmosphere_;
  const double total_pa = atm.total_pressure_pa();
  const double true_o2_pct = atm.o2_fraction() * 100.0;
  const double o2_pct = drills_.active("o2_sensor_stuck") ?
    drills_.param("value_pct", 23.0) : true_o2_pct;

  CabinAtmosphereMsg m;
  m.stamp = now;
  m.total_pressure_kpa = units::pa_to_kpa(total_pa);
  m.temperature_c = units::kelvin_to_celsius(atm.temperature_k());
  m.relative_humidity = atm.relative_humidity();
  m.o2_partial_pressure_kpa = units::pa_to_kpa(o2_pct / 100.0 * total_pa);
  m.co2_partial_pressure_kpa = units::pa_to_kpa(atm.partial_pressure_pa(cabin::Gas::CO2));
  m.h2o_partial_pressure_kpa = units::pa_to_kpa(atm.water_partial_pressure_pa());
  atmosphere_pub_->publish(m);

  std_msgs::msg::Float64 f;
  f.data = atm.co2_ppm();
  co2_pub_->publish(f);
  f.data = units::kg_per_s_to_kg_per_day(last_control_.condensate_kg_s);
  condensate_pub_->publish(f);

  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = now;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "cabin";
  status.hardware_id = "ssos_eclss/cabin";
  auto kv = [&status](const std::string & k, double v) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = k;
      pair.value = std::to_string(v);
      status.values.push_back(pair);
    };
  kv("co2_ppm", atm.co2_ppm());
  kv("o2_fraction", o2_pct / 100.0);
  kv("o2_fraction_mca", true_o2_pct / 100.0);
  kv("total_pressure_kpa", m.total_pressure_kpa);
  kv("relative_humidity", m.relative_humidity);
  kv("temperature_c", m.temperature_c);
  kv("crew_co2_kg_day", units::kg_per_s_to_kg_per_day(crew_co2_.get()));
  kv("crew_o2_kg_day", units::kg_per_s_to_kg_per_day(crew_o2_.get()));
  kv("ars_co2_removal_kg_day", units::kg_per_s_to_kg_per_day(ars_removal_.get()));
  kv("ogs_o2_kg_day", units::kg_per_s_to_kg_per_day(ogs_o2_.get()));
  kv("condensate_kg_day", f.data);
  kv("leak_kg_day", units::kg_per_s_to_kg_per_day(leak_kg_s_));
  kv("n2_makeup_kg_day", units::kg_per_s_to_kg_per_day(last_control_.n2_makeup_kg_s));
  kv("o2_makeup_kg_day", units::kg_per_s_to_kg_per_day(last_control_.o2_makeup_kg_s));
  kv("n2_tank_kg", control_.n2_tank_kg());
  kv("o2_tank_kg", control_.o2_tank_kg());
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = drills_.tracker().any() ? "drill: " + drills_.tracker().type() : "nominal";
  diag.status.push_back(status);
  telemetry_pub_->publish(diag);
}

void CabinNode::check_health(const rclcpp::Time & now)
{
  std::string fault, msg = "nominal";
  uint8_t severity = FaultEvent::SEVERITY_CRITICAL;
  if (enable_auto_faults_) {
    const auto & atm = *atmosphere_;
    const double o2_pct = atm.o2_fraction() * 100.0;
    const double sensor_pct = drills_.active("o2_sensor_stuck") ?
      drills_.param("value_pct", 23.0) : o2_pct;
    const double p_kpa = units::pa_to_kpa(atm.total_pressure_pa());
    const double rh = atm.relative_humidity();
    if (units::kg_per_s_to_kg_per_day(leak_kg_s_) > leak_alarm_kg_day_) {
      fault = "cabin_leak";
      msg = "cabin leak rate above limit";
    } else if (p_kpa < pressure_low_kpa_ || p_kpa > pressure_high_kpa_) {
      fault = "cabin_pressure";
      msg = "cabin pressure out of limits";
    } else if (atm.co2_ppm() > co2_alarm_ppm_) {
      fault = "co2_high";
      msg = "cabin CO2 above alarm threshold";
    } else if (o2_pct < o2_low_pct_ || o2_pct > o2_high_pct_) {
      fault = o2_pct < o2_low_pct_ ? "o2_low" : "o2_high";
      msg = "cabin O2 out of limits";
    } else if (std::abs(sensor_pct - o2_pct) > o2_sensor_tolerance_pct_) {
      fault = "o2_sensor_disagree";
      msg = "O2 sensor disagrees with the major constituent analyser";
      severity = FaultEvent::SEVERITY_WARNING;
    } else if (rh < rh_low_ || rh > rh_high_) {
      fault = "humidity";
      msg = "cabin humidity out of limits";
      severity = FaultEvent::SEVERITY_WARNING;
    }
  }
  health_.report(now, fault, msg, severity, {"/ssos/cabin/atmosphere"});
}

}  // namespace nodes
}  // namespace ssos_eclss
