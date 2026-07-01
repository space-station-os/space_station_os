#include "ssos_eclss/nodes/crew_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

#include "ssos_eclss/common/units.hpp"
#include "ssos_eclss/nodes/eclss_diagnostics.hpp"

using std::placeholders::_1;

namespace ssos_eclss
{
namespace nodes
{

CrewNode::CrewNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("crew_node", options)
{
  this->declare_parameter("step_rate_hz", step_rate_hz_);
  this->declare_parameter("day_length_s", day_length_s_);
  this->declare_parameter("metabolic_scale", metabolic_scale_);
  this->declare_parameter("crew_size", crew_size_);
  this->declare_parameter("drink_kg_day", drink_kg_day_);
  this->declare_parameter("flush_kg_day", flush_kg_day_);
  this->declare_parameter("urine_kg_day", urine_kg_day_);
  this->declare_parameter("feces_water_kg_day", feces_water_kg_day_);
  this->declare_parameter("trash_water_kg_day", trash_water_kg_day_);
  autostart_timer_ = EclssDiagnostics::maybe_autostart(this);
}

crew::CrewParams CrewNode::build_params() const
{
  crew::CrewParams p = crew::default_crew_params();
  p.crew_size = crew_size_;
  p.drink_kg_day = drink_kg_day_;
  p.flush_kg_day = flush_kg_day_;
  p.urine_kg_day = urine_kg_day_;
  p.feces_water_kg_day = feces_water_kg_day_;
  p.trash_water_kg_day = trash_water_kg_day_;
  for (auto & a : p.schedule) {
    a.co2_g_min *= metabolic_scale_;
    a.o2_g_min *= metabolic_scale_;
    a.latent_g_min *= metabolic_scale_;
  }
  return p;
}

CallbackReturn CrewNode::on_configure(const rclcpp_lifecycle::State &)
{
  step_rate_hz_ = this->get_parameter("step_rate_hz").as_double();
  day_length_s_ = this->get_parameter("day_length_s").as_double();
  metabolic_scale_ = this->get_parameter("metabolic_scale").as_double();
  crew_size_ = static_cast<int>(this->get_parameter("crew_size").as_int());
  drink_kg_day_ = this->get_parameter("drink_kg_day").as_double();
  flush_kg_day_ = this->get_parameter("flush_kg_day").as_double();
  urine_kg_day_ = this->get_parameter("urine_kg_day").as_double();
  feces_water_kg_day_ = this->get_parameter("feces_water_kg_day").as_double();
  trash_water_kg_day_ = this->get_parameter("trash_water_kg_day").as_double();

  crew_ = std::make_unique<crew::CrewSimulator>(build_params());

  co2_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ssos/crew/co2_kg_day", 10);
  o2_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/ssos/crew/o2_consumption_kg_day", 10);
  latent_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/ssos/crew/latent_water_kg_day", 10);
  urine_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/ssos/crew/urine_kg_day", 10);
  potable_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/ssos/crew/potable_demand_kg_day", 10);
  telemetry_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/ssos/crew/diagnostics", 10);
  heartbeat_pub_ =
    this->create_publisher<SubsystemHeartbeat>("/ssos/crew/heartbeat", 10);
  register_client_ =
    this->create_client<RegisterSubsystem>("/ssos/register_subsystem");
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CrewNode::on_set_parameters, this, _1));
  return CallbackReturn::SUCCESS;
}

CallbackReturn CrewNode::on_activate(const rclcpp_lifecycle::State &)
{
  co2_pub_->on_activate();
  o2_pub_->on_activate();
  latent_pub_->on_activate();
  urine_pub_->on_activate();
  potable_pub_->on_activate();
  telemetry_pub_->on_activate();
  heartbeat_pub_->on_activate();
  const auto period = std::chrono::duration<double>(1.0 / std::max(step_rate_hz_, 1.0e-3));
  step_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&CrewNode::step, this));
  register_with_manager();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CrewNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (step_timer_) {
    step_timer_->cancel();
    step_timer_.reset();
  }
  co2_pub_->on_deactivate();
  o2_pub_->on_deactivate();
  latent_pub_->on_deactivate();
  urine_pub_->on_deactivate();
  potable_pub_->on_deactivate();
  telemetry_pub_->on_deactivate();
  heartbeat_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CrewNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  co2_pub_.reset();
  o2_pub_.reset();
  latent_pub_.reset();
  urine_pub_.reset();
  potable_pub_.reset();
  telemetry_pub_.reset();
  heartbeat_pub_.reset();
  register_client_.reset();
  crew_.reset();
  return CallbackReturn::SUCCESS;
}

void CrewNode::step()
{
  const double day_fraction =
    this->now().seconds() / std::max(day_length_s_, 1.0);
  const crew::CrewOutputs out = crew_->outputs(day_fraction);
  last_outputs_ = out;

  auto pub = [](auto & p, double v) {
    std_msgs::msg::Float64 m;
    m.data = v;
    p->publish(m);
  };
  pub(co2_pub_, units::kg_per_s_to_kg_per_day(out.co2_kg_s));
  pub(o2_pub_, units::kg_per_s_to_kg_per_day(out.o2_consumption_kg_s));
  pub(latent_pub_, units::kg_per_s_to_kg_per_day(out.latent_water_kg_s));
  pub(urine_pub_, units::kg_per_s_to_kg_per_day(out.urine_kg_s));
  pub(potable_pub_, units::kg_per_s_to_kg_per_day(out.potable_demand_kg_s));

  const rclcpp::Time now = this->now();
  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = now;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "crew";
  status.hardware_id = "ssos_eclss/crew";
  auto kv = [&](const std::string & k, const std::string & v) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = k;
    pair.value = v;
    status.values.push_back(pair);
  };
  kv("activity", out.activity);
  kv("crew_size", std::to_string(crew_size_));
  kv("co2_kg_day", std::to_string(out.co2_kg_day));
  kv("o2_kg_day", std::to_string(out.o2_kg_day));
  kv("latent_water_kg_day",
     std::to_string(units::kg_per_s_to_kg_per_day(out.latent_water_kg_s)));
  kv("urine_kg_day", std::to_string(units::kg_per_s_to_kg_per_day(out.urine_kg_s)));
  kv("potable_demand_kg_day",
     std::to_string(units::kg_per_s_to_kg_per_day(out.potable_demand_kg_s)));
  kv("metabolic_heat_w", std::to_string(out.metabolic_heat_w));
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = out.activity;
  diag.status.push_back(status);
  telemetry_pub_->publish(diag);

  heartbeat_pub_->publish(EclssDiagnostics::make_heartbeat(
    now, "crew", SubsystemHeartbeat::LIFECYCLE_ACTIVE, true, out.activity));
}

void CrewNode::register_with_manager()
{
  if (!register_client_->wait_for_service(std::chrono::milliseconds(200))) {
    RCLCPP_WARN(get_logger(), "system_manager unavailable; continuing");
    return;
  }
  auto req = std::make_shared<RegisterSubsystem::Request>();
  req->subsystem_name = "crew";
  req->published_topics = {"/ssos/crew/co2_kg_day", "/ssos/crew/o2_consumption_kg_day",
                           "/ssos/crew/latent_water_kg_day", "/ssos/crew/urine_kg_day",
                           "/ssos/crew/potable_demand_kg_day", "/ssos/crew/diagnostics"};
  req->subscribed_topics = {};
  req->heartbeat_topic = "/ssos/crew/heartbeat";
  register_client_->async_send_request(req);
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
      crew_size_ = static_cast<int>(p.as_int());
    } else if (n == "drink_kg_day") {
      drink_kg_day_ = p.as_double();
    } else if (n == "flush_kg_day") {
      flush_kg_day_ = p.as_double();
    } else if (n == "urine_kg_day") {
      urine_kg_day_ = p.as_double();
    } else if (n == "feces_water_kg_day") {
      feces_water_kg_day_ = p.as_double();
    } else if (n == "trash_water_kg_day") {
      trash_water_kg_day_ = p.as_double();
    } else if (n == "day_length_s") {
      day_length_s_ = p.as_double();
    } else {
      continue;
    }
  }
  if (crew_) {
    crew_->set_params(build_params());
  }
  return result;
}

}  // namespace nodes
}  // namespace ssos_eclss
