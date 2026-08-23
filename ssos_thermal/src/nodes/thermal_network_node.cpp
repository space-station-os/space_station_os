#include "ssos_thermal/nodes/thermal_network_node.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"

using std::placeholders::_1;

namespace ssos_thermal
{
namespace nodes
{

ThermalNetworkNode::ThermalNetworkNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("thermal_network", options)
{
  this->declare_parameter("enable_failure", enable_failure_);
  this->declare_parameter("enable_cooling", enable_cooling_);
  this->declare_parameter("cooling_trigger_threshold", cooling_trigger_threshold_);
  this->declare_parameter("max_temp_threshold", max_temp_threshold_);
  this->declare_parameter("cooling_rate", cooling_rate_);
  this->declare_parameter("thermal_update_dt", thermal_update_dt_);
  this->declare_parameter("thermal_config_file", thermal_config_file_);

  autostart_timer_ = ThermalDiagnostics::maybe_autostart(this);
}

CallbackReturn ThermalNetworkNode::on_configure(const rclcpp_lifecycle::State &)
{
  enable_failure_ = this->get_parameter("enable_failure").as_bool();
  enable_cooling_ = this->get_parameter("enable_cooling").as_bool();
  cooling_trigger_threshold_ = this->get_parameter("cooling_trigger_threshold").as_double();
  max_temp_threshold_ = this->get_parameter("max_temp_threshold").as_double();
  cooling_rate_ = this->get_parameter("cooling_rate").as_double();
  thermal_update_dt_ = this->get_parameter("thermal_update_dt").as_double();
  thermal_config_file_ = this->get_parameter("thermal_config_file").as_string();
  cooling_active_ = false;

  const std::string share_dir =
    ament_index_cpp::get_package_share_directory("ssos_thermal");
  const std::string config_path = share_dir + "/" + thermal_config_file_;
  network_ = std::make_unique<network::ThermalNetwork>(
    network::ThermalNetwork::load_from_yaml(config_path));

  node_pub_ = this->create_publisher<space_station_interfaces::msg::ThermalNodeDataArray>(
    "/thermal/nodes/state", 10);
  link_pub_ = this->create_publisher<space_station_interfaces::msg::ThermalLinkFlowsArray>(
    "/thermal/links/flux", 10);
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "/thermals/diagnostics", 10);
  heartbeat_pub_ = this->create_publisher<SubsystemHeartbeat>("/ssos/thermal/heartbeat", 10);
  fault_pub_ = this->create_publisher<FaultEvent>("/ssos/fault_event", 10);

  cooling_client_ = rclcpp_action::create_client<space_station_interfaces::action::Coolant>(
    this, "/coolant_heat_transfer");
  register_client_ = this->create_client<RegisterSubsystem>("/ssos/register_subsystem");

  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ThermalNetworkNode::onSetParameters, this, _1));

  RCLCPP_INFO(get_logger(), "Thermal network configured (%zu nodes, %zu links)",
              network_->nodes().size(), network_->links().size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn ThermalNetworkNode::on_activate(const rclcpp_lifecycle::State &)
{
  node_pub_->on_activate();
  link_pub_->on_activate();
  diag_pub_->on_activate();
  heartbeat_pub_->on_activate();
  fault_pub_->on_activate();

  step_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(thermal_update_dt_),
    std::bind(&ThermalNetworkNode::updateSimulation, this));

  registerWithManager();
  RCLCPP_INFO(get_logger(), "Thermal network activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ThermalNetworkNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (step_timer_) {
    step_timer_->cancel();
    step_timer_.reset();
  }
  node_pub_->on_deactivate();
  link_pub_->on_deactivate();
  diag_pub_->on_deactivate();
  heartbeat_pub_->on_deactivate();
  fault_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ThermalNetworkNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  node_pub_.reset();
  link_pub_.reset();
  diag_pub_.reset();
  heartbeat_pub_.reset();
  fault_pub_.reset();
  cooling_client_.reset();
  register_client_.reset();
  network_.reset();
  return CallbackReturn::SUCCESS;
}

void ThermalNetworkNode::coolingCallback()
{
  using namespace space_station_interfaces::action;

  const double avg_temp = network_->average_temperature();
  if (cooling_active_ || !enable_cooling_ || avg_temp <= cooling_trigger_threshold_) {
    return;
  }

  if (!cooling_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(get_logger(), "[COOLING] Cooling action server unavailable.");
    return;
  }

  Coolant::Goal goal_msg;
  goal_msg.input_temperature_c = avg_temp;
  goal_msg.component_id = "thermal_network";

  RCLCPP_INFO(get_logger(), "[COOLING] Sending goal: avg_temperature = %.2f degC", avg_temp);

  auto send_goal_options = rclcpp_action::Client<Coolant>::SendGoalOptions();
  send_goal_options.feedback_callback =
    [this](GoalHandleCoolant::SharedPtr,
           const std::shared_ptr<const Coolant::Feedback> feedback) {
      RCLCPP_INFO(get_logger(),
        "[COOLING][Feedback] Internal=%.2f degC | Ammonia=%.2f degC | Vented=%.2f kJ",
        feedback->internal_temp_c, feedback->ammonia_temp_c, feedback->vented_heat_kj);
      feedback_latest_temp_ = feedback->internal_temp_c;
      network_->set_all_temperatures(feedback_latest_temp_);
    };
  send_goal_options.result_callback =
    [this](const GoalHandleCoolant::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(get_logger(), "[COOLING] Completed: %s", result.result->message.c_str());
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(get_logger(), "[COOLING] Aborted: %s", result.result->message.c_str());
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(get_logger(), "[COOLING] Canceled.");
          break;
        default:
          RCLCPP_ERROR(get_logger(), "[COOLING] Unknown result code.");
          break;
      }
      cooling_active_ = false;
    };

  cooling_client_->async_send_goal(goal_msg, send_goal_options);
  cooling_active_ = true;
}

void ThermalNetworkNode::updateSimulation()
{
  if (network_->nodes().empty()) {
    return;
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000,
                       "Avg temperature = %.2f degC", network_->average_temperature());

  coolingCallback();

  if (!cooling_active_) {
    network_->step(thermal_update_dt_);
  }

  const network::ThermalNetwork::Hottest hottest = network_->hottest();

  space_station_interfaces::msg::ThermalNodeDataArray node_msg;
  for (const auto & [name, node] : network_->nodes()) {
    space_station_interfaces::msg::ThermalNodeData data;
    data.name = name;
    data.temperature = node.temperature;
    data.heat_capacity = node.heat_capacity;
    data.internal_power = node.internal_power;
    node_msg.nodes.push_back(data);
  }
  node_pub_->publish(node_msg);
  publishThermalNetworkDiag(hottest);

  space_station_interfaces::msg::ThermalLinkFlowsArray link_msg;
  for (const auto & link : network_->links()) {
    space_station_interfaces::msg::ThermalLinkFlows l;
    l.node_a = link.from;
    l.node_b = link.to;
    l.conductance = link.conductance;

    const double t_a = network_->node_temperature(link.joint_name);
    const double t_b = 20.0;  // Reference temp in Celsius
    l.heat_flow = link.conductance * (t_a - t_b);
    link_msg.links.push_back(l);
  }
  link_pub_->publish(link_msg);

  const rclcpp::Time now = this->now();
  const bool healthy = !(enable_failure_ && hottest.temperature > max_temp_threshold_);
  const std::string status_message = healthy ?
    "nominal" : "overheating: " + hottest.name;

  if (diag_.should_raise_fault(healthy)) {
    fault_pub_->publish(ThermalDiagnostics::make_fault(
      now, "thermal", "thermal_node_overheat", FaultEvent::SEVERITY_CRITICAL,
      status_message, {"/thermal/nodes/state"}));
  }
  heartbeat_pub_->publish(ThermalDiagnostics::make_heartbeat(
    now, "thermal", SubsystemHeartbeat::LIFECYCLE_ACTIVE, healthy, status_message));
}

void ThermalNetworkNode::publishThermalNetworkDiag(
  const network::ThermalNetwork::Hottest & hottest)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using diagnostic_msgs::msg::KeyValue;

  DiagnosticStatus st;
  st.name = "ThermalNetwork";

  if (std::isfinite(hottest.temperature) && hottest.temperature > max_temp_threshold_) {
    st.level = DiagnosticStatus::WARN;
    st.message = "Node temperature exceeds max_temp_threshold";
  } else {
    st.level = DiagnosticStatus::OK;
    st.message = "All node temperatures within threshold";
  }

  KeyValue kv1, kv2;
  kv1.key = "hottest_node";
  kv1.value = hottest.name;

  kv2.key = "node_temperature_C";
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << hottest.temperature;
    kv2.value = oss.str();
  }

  st.hardware_id = hottest.name;
  st.values = {kv1, kv2};
  diag_pub_->publish(st);
}

void ThermalNetworkNode::registerWithManager()
{
  if (!register_client_->wait_for_service(std::chrono::milliseconds(200))) {
    RCLCPP_WARN(get_logger(),
                "system_manager registration service unavailable; continuing");
    return;
  }
  auto req = std::make_shared<RegisterSubsystem::Request>();
  req->subsystem_name = "thermal";
  req->published_topics = {"/thermal/nodes/state", "/thermal/links/flux",
                           "/thermals/diagnostics"};
  req->subscribed_topics = {};
  req->heartbeat_topic = "/ssos/thermal/heartbeat";
  register_client_->async_send_request(req);
}

rcl_interfaces::msg::SetParametersResult ThermalNetworkNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  for (const auto & param : params) {
    const std::string & name = param.get_name();
    if (name == "enable_failure") {
      enable_failure_ = param.as_bool();
    } else if (name == "enable_cooling") {
      enable_cooling_ = param.as_bool();
    } else if (name == "cooling_trigger_threshold") {
      cooling_trigger_threshold_ = param.as_double();
    } else if (name == "max_temp_threshold") {
      max_temp_threshold_ = param.as_double();
    } else if (name == "cooling_rate") {
      cooling_rate_ = param.as_double();
    } else if (name == "thermal_update_dt") {
      thermal_update_dt_ = param.as_double();
      if (step_timer_) {
        step_timer_->cancel();
        step_timer_ = this->create_wall_timer(
          std::chrono::duration<double>(thermal_update_dt_),
          std::bind(&ThermalNetworkNode::updateSimulation, this));
      }
    }
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

}  // namespace nodes
}  // namespace ssos_thermal
