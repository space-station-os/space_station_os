#include "ssos_eclss/nodes/node_support.hpp"

#include <algorithm>
#include <utility>

#include "ssos_eclss/nodes/eclss_diagnostics.hpp"

namespace ssos_eclss
{
namespace nodes
{

using space_station_interfaces::msg::FaultEvent;
using space_station_interfaces::msg::SubsystemHeartbeat;
using space_station_interfaces::srv::RegisterSubsystem;

double StepClock::advance(const rclcpp::Time & now)
{
  if (!started_ || now.get_clock_type() != last_.get_clock_type()) {
    started_ = true;
    last_ = now;
    return 0.0;
  }
  const double dt = (now - last_).seconds();
  last_ = now;
  capped_ = dt > max_step_s_;
  return dt > 0.0 ? std::min(dt, max_step_s_) : 0.0;
}

namespace
{
constexpr double kInputTimeoutS = 5.0;
}

void LatchedValue::subscribe(rclcpp_lifecycle::LifecycleNode * node, const std::string & topic)
{
  reset();
  sub_ = node->create_subscription<std_msgs::msg::Float64>(
    topic, 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
      value_ = msg->data * scale_;
      stamp_ = wall_.now();
      heard_ = true;
    });
}

void LatchedValue::reset()
{
  sub_.reset();
  value_ = 0.0;
  heard_ = false;
}

bool LatchedValue::fresh() const
{
  return heard_ && (wall_.now() - stamp_).seconds() < kInputTimeoutS;
}

double LatchedValue::get() const
{
  if (fresh() || !fallback_) {
    return value_;
  }
  return *fallback_;
}

InjectedFaults::InjectedFaults(rclcpp_lifecycle::LifecycleNode * node, std::string subsystem)
: node_(node), subsystem_(std::move(subsystem))
{}

void InjectedFaults::start()
{
  // The drill belongs to the subsystem until it is cleared or expires, so it
  // survives reconfiguration; report a change on the first update so the node
  // applies it (or its nominal configuration) to the freshly built physics.
  seen_revision_ = tracker_.revision() - 1;
  sub_ = node_->create_subscription<FaultEvent>(
    "/ssos/fault_event", 20,
    [this](const FaultEvent::SharedPtr msg) {on_fault(msg);});
}

void InjectedFaults::stop()
{
  sub_.reset();
}

void InjectedFaults::on_fault(const FaultEvent::SharedPtr msg)
{
  if (msg->subsystem_name != subsystem_) {
    return;
  }
  const auto fault = ssos_sim::parse_injected_fault(msg->fault_type, msg->description);
  if (!fault) {
    return;  // the subsystem's own fault report, not a drill
  }
  if (fault->type != ssos_sim::kClearFaults &&
    !ssos_sim::is_known_drill(subsystem_, fault->type))
  {
    RCLCPP_WARN(node_->get_logger(), "Ignoring unknown %s drill '%s'", subsystem_.c_str(),
                fault->type.c_str());
    return;
  }
  tracker_.apply(*fault, node_->now().seconds());
  RCLCPP_WARN(node_->get_logger(), "Drill %s: %s", fault->type == ssos_sim::kClearFaults ?
              "cleared" : "started", fault->type.c_str());
}

bool InjectedFaults::update(const rclcpp::Time & now)
{
  if (tracker_.expire(now.seconds())) {
    RCLCPP_INFO(node_->get_logger(), "Drill ended");
  }
  const bool changed = tracker_.revision() != seen_revision_;
  seen_revision_ = tracker_.revision();
  return changed;
}

HealthReporter::HealthReporter(rclcpp_lifecycle::LifecycleNode * node, std::string subsystem,
                               std::vector<std::string> published,
                               std::vector<std::string> subscribed)
: node_(node), subsystem_(std::move(subsystem)), published_(std::move(published)),
  subscribed_(std::move(subscribed))
{}

void HealthReporter::configure()
{
  heartbeat_pub_ = node_->create_publisher<SubsystemHeartbeat>(
    "/ssos/" + subsystem_ + "/heartbeat", 10);
  fault_pub_ = node_->create_publisher<FaultEvent>("/ssos/fault_event", 10);
  register_client_ = node_->create_client<RegisterSubsystem>("/ssos/register_subsystem");
  registered_ = false;
  register_pending_ = false;
  last_fault_.clear();
}

void HealthReporter::activate()
{
  heartbeat_pub_->on_activate();
  fault_pub_->on_activate();
  last_fault_.clear();
}

void HealthReporter::deactivate()
{
  heartbeat_pub_->on_deactivate();
  fault_pub_->on_deactivate();
}

void HealthReporter::cleanup()
{
  heartbeat_pub_.reset();
  fault_pub_.reset();
  register_client_.reset();
}

void HealthReporter::try_register()
{
  // A request with no answer (system_manager restarted) is retried.
  if (register_pending_ && (wall_.now() - register_sent_).seconds() > 5.0) {
    register_pending_ = false;
  }
  if (registered_ || register_pending_ || !register_client_->service_is_ready()) {
    return;
  }
  auto req = std::make_shared<RegisterSubsystem::Request>();
  req->subsystem_name = subsystem_;
  req->published_topics = published_;
  req->subscribed_topics = subscribed_;
  req->heartbeat_topic = "/ssos/" + subsystem_ + "/heartbeat";
  register_pending_ = true;
  register_sent_ = wall_.now();
  register_client_->async_send_request(
    req, [this](rclcpp::Client<RegisterSubsystem>::SharedFuture future) {
      register_pending_ = false;
      registered_ = future.get()->accepted;
    });
}

void HealthReporter::report(const rclcpp::Time & now, const std::string & fault_type,
                            const std::string & message, uint8_t severity,
                            const std::vector<std::string> & interfaces)
{
  const bool healthy = fault_type.empty();
  heartbeat_pub_->publish(EclssDiagnostics::make_heartbeat(
    now, subsystem_, SubsystemHeartbeat::LIFECYCLE_ACTIVE, healthy,
    message));
  if (!healthy && fault_type != last_fault_) {
    fault_pub_->publish(EclssDiagnostics::make_fault(
      now, subsystem_, fault_type, severity, message, interfaces));
  }
  last_fault_ = fault_type;
  // Heartbeat first, so system_manager never sees a registered subsystem
  // without one.
  try_register();
}

}  // namespace nodes
}  // namespace ssos_eclss
