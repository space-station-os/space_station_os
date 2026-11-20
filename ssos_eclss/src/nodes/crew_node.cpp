#include "ssos_eclss/nodes/crew_node.hpp"

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

CrewNode::CrewNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("crew_node", options),
  drills_(this, "crew"),
  health_(this, "crew",
    {"/ssos/crew/co2_kg_day", "/ssos/crew/o2_consumption_kg_day",
      "/ssos/crew/latent_water_kg_day", "/ssos/crew/urine_kg_day",
      "/ssos/crew/potable_demand_kg_day", "/ssos/crew/diagnostics", "/thermal/heat_loads"},
    {})
{
  const auto d = crew::default_crew_params();
  this->declare_parameter("step_rate_hz", step_rate_hz_);
  this->declare_parameter("initial_time_of_day_hr", initial_hour_);
  this->declare_parameter("metabolic_scale", metabolic_scale_);
  this->declare_parameter("crew_size", crew_size_);
  this->declare_parameter("thermal_node", thermal_node_);
  this->declare_parameter("sleep_start_hr", d.sleep_start_hr);
  this->declare_parameter("sleep_hours", d.sleep_hours);
  this->declare_parameter("first_exercise_hr", d.first_exercise_hr);
  this->declare_parameter("exercise_spacing_hr", d.exercise_spacing_hr);
  this->declare_parameter("drink_kg_day", d.drink_kg_day);
  this->declare_parameter("flush_kg_day", d.flush_kg_day);
  this->declare_parameter("urine_kg_day", d.urine_kg_day);
  this->declare_parameter("feces_water_kg_day", d.feces_water_kg_day);
  this->declare_parameter("trash_water_kg_day", d.trash_water_kg_day);
  autostart_timer_ = EclssDiagnostics::maybe_autostart(this);
}

crew::CrewParams CrewNode::build_params() const
{
  crew::CrewParams p = crew::default_crew_params();
  p.crew_size = crew_size_;
  p.sleep_start_hr = this->get_parameter("sleep_start_hr").as_double();
  p.sleep_hours = this->get_parameter("sleep_hours").as_double();
  p.first_exercise_hr = this->get_parameter("first_exercise_hr").as_double();
  p.exercise_spacing_hr = this->get_parameter("exercise_spacing_hr").as_double();
  p.drink_kg_day = this->get_parameter("drink_kg_day").as_double();
  p.flush_kg_day = this->get_parameter("flush_kg_day").as_double();
  p.urine_kg_day = this->get_parameter("urine_kg_day").as_double();
  p.feces_water_kg_day = this->get_parameter("feces_water_kg_day").as_double();
  p.trash_water_kg_day = this->get_parameter("trash_water_kg_day").as_double();
  return p;
}

CallbackReturn CrewNode::on_configure(const rclcpp_lifecycle::State &)
{
  step_rate_hz_ = this->get_parameter("step_rate_hz").as_double();
  initial_hour_ = this->get_parameter("initial_time_of_day_hr").as_double();
  metabolic_scale_ = this->get_parameter("metabolic_scale").as_double();
  crew_size_ = static_cast<int>(this->get_parameter("crew_size").as_int());
  thermal_node_ = this->get_parameter("thermal_node").as_string();
  crew_.set_params(build_params());

  co2_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ssos/crew/co2_kg_day", 10);
  o2_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/ssos/crew/o2_consumption_kg_day", 10);
  latent_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/ssos/crew/latent_water_kg_day", 10);
  urine_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ssos/crew/urine_kg_day", 10);
  potable_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/ssos/crew/potable_demand_kg_day", 10);
  telemetry_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/ssos/crew/diagnostics", 10);
  heat_pub_ = this->create_publisher<space_station_interfaces::msg::HeatLoadArray>(
    "/thermal/heat_loads", 10);
  health_.configure();
  drills_.start();
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CrewNode::on_set_parameters, this, _1));
  return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult CrewNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : params) {
    const std::string & n = p.get_name();
    if (n == "metabolic_scale") {
      metabolic_scale_ = p.as_double();
    } else if (n == "crew_size") {
      if (p.as_int() < 0) {
        result.successful = false;
        result.reason = "crew_size must not be negative";
        return result;
      }
      crew_size_ = static_cast<int>(p.as_int());
      auto cp = crew_.params();
      cp.crew_size = crew_size_;
      crew_.set_params(cp);
    }
  }
  return result;
}

CallbackReturn CrewNode::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto * pub : {&co2_pub_, &o2_pub_, &latent_pub_, &urine_pub_, &potable_pub_}) {
    (*pub)->on_activate();
  }
  telemetry_pub_->on_activate();
  heat_pub_->on_activate();
  health_.activate();
  const auto period = std::chrono::duration<double>(1.0 / std::max(step_rate_hz_, 1.0e-3));
  step_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&CrewNode::step, this));
  return CallbackReturn::SUCCESS;
}

CallbackReturn CrewNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  for (auto * pub : {&co2_pub_, &o2_pub_, &latent_pub_, &urine_pub_, &potable_pub_}) {
    (*pub)->on_deactivate();
  }
  telemetry_pub_->on_deactivate();
  heat_pub_->on_deactivate();
  health_.deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CrewNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  for (auto * pub : {&co2_pub_, &o2_pub_, &latent_pub_, &urine_pub_, &potable_pub_}) {
    pub->reset();
  }
  telemetry_pub_.reset();
  heat_pub_.reset();
  health_.cleanup();
  drills_.stop();
  return CallbackReturn::SUCCESS;
}

void CrewNode::step()
{
  const rclcpp::Time now = this->now();
  drills_.update(now);
  const bool surge = drills_.active("crew_metabolic_surge");
  const double scale = metabolic_scale_ * (surge ? drills_.param("scale", 2.0) : 1.0);
  const double hour = initial_hour_ + now.seconds() / units::SECONDS_PER_HOUR;
  last_outputs_ = crew_.outputs(hour, scale);
  const auto & out = last_outputs_;

  auto pub = [](FloatPub & p, double kg_s) {
      std_msgs::msg::Float64 m;
      m.data = units::kg_per_s_to_kg_per_day(kg_s);
      p->publish(m);
    };
  pub(co2_pub_, out.co2_kg_s);
  pub(o2_pub_, out.o2_consumption_kg_s);
  pub(latent_pub_, out.latent_water_kg_s);
  pub(urine_pub_, out.urine_kg_s);
  pub(potable_pub_, out.potable_demand_kg_s);

  space_station_interfaces::msg::HeatLoadArray heat;
  heat.header.stamp = now;
  heat.source = "crew";
  space_station_interfaces::msg::HeatLoad load;
  load.node_name = thermal_node_;
  load.power = out.metabolic_heat_w;
  heat.loads.push_back(load);
  heat_pub_->publish(heat);

  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = now;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "crew";
  status.hardware_id = "ssos_eclss/crew";
  auto kv = [&status](const std::string & k, const std::string & v) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = k;
      pair.value = v;
      status.values.push_back(pair);
    };
  kv("activity", crew::to_string(out.activity));
  kv("crew_size", std::to_string(crew_size_));
  kv("exercising", std::to_string(out.exercising));
  kv("time_of_day_hr", std::to_string(std::fmod(hour, 24.0)));
  kv("co2_kg_day", std::to_string(units::kg_per_s_to_kg_per_day(out.co2_kg_s)));
  kv("o2_kg_day", std::to_string(units::kg_per_s_to_kg_per_day(out.o2_consumption_kg_s)));
  kv("latent_water_kg_day",
    std::to_string(units::kg_per_s_to_kg_per_day(out.latent_water_kg_s)));
  kv("urine_kg_day", std::to_string(units::kg_per_s_to_kg_per_day(out.urine_kg_s)));
  kv("potable_demand_kg_day",
    std::to_string(units::kg_per_s_to_kg_per_day(out.potable_demand_kg_s)));
  kv("metabolic_heat_w", std::to_string(out.metabolic_heat_w));
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = crew::to_string(out.activity);
  diag.status.push_back(status);
  telemetry_pub_->publish(diag);

  health_.report(now, surge ? "crew_metabolic_surge" : "",
    surge ? "crew metabolic load above plan" : crew::to_string(out.activity),
    FaultEvent::SEVERITY_WARNING);
}

}  // namespace nodes
}  // namespace ssos_eclss
