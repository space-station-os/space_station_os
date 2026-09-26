#include "ssos_gnc/nodes/attitude_control_node.hpp"

#include <chrono>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace ssos_gnc
{

namespace nodes
{

namespace
{
geometry_msgs::msg::Vector3 to_vector3(const Eigen::Vector3d & v)
{
  geometry_msgs::msg::Vector3 m;
  m.x = v.x();
  m.y = v.y();
  m.z = v.z();
  return m;
}  // namespace nodes

geometry_msgs::msg::Quaternion to_quaternion(const common::Quaternion & q)
{
  geometry_msgs::msg::Quaternion m;
  m.x = q.x();
  m.y = q.y();
  m.z = q.z();
  m.w = q.w();
  return m;
}  // namespace ssos_gnc

std_msgs::msg::Float64MultiArray to_array(const Eigen::VectorXd & v)
{
  std_msgs::msg::Float64MultiArray m;
  m.data.reserve(static_cast<std::size_t>(v.size()));
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    m.data.push_back(v(i));
  }
  return m;
}
}

AttitudeControlNode::AttitudeControlNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("attitude_control", options)
{
  declare_parameter("subsystem_name", subsystem_name_);
  declare_parameter("step_rate_hz", step_rate_hz_);
  declare_parameter("heartbeat_rate_hz", heartbeat_rate_hz_);
  declare_parameter("thrusters.table_path", std::string(""));
  declare_parameter("thrusters.table_name", std::string("sixdof_phys"));
}

CallbackReturn AttitudeControlNode::on_configure(const rclcpp_lifecycle::State &)
{
  subsystem_name_ = get_parameter("subsystem_name").as_string();
  step_rate_hz_ = get_parameter("step_rate_hz").as_double();
  heartbeat_rate_hz_ = get_parameter("heartbeat_rate_hz").as_double();

  if (step_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "step_rate_hz must be positive");
    return CallbackReturn::FAILURE;
  }

  bridge_ = std::make_unique<GncParameterBridge>(this);
  bridge_->declare_flight_parameters(flight::default_gnc_parameters());

  system_ = std::make_unique<flight::AttitudeControlSystem>(bridge_->read_flight_parameters());

  const std::string table_path = get_parameter("thrusters.table_path").as_string();
  const std::string table_name = get_parameter("thrusters.table_name").as_string();
  if (!table_path.empty()) {
    const LoadResult load = loader_.load_table(table_path, table_name);
    if (load.success && system_->configure_thrusters(loader_.geometry())) {
      RCLCPP_INFO(get_logger(), "%s", load.message.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(), "thruster geometry unavailable (%s); CMG-only operation",
        load.message.c_str());
    }
  } else {
    RCLCPP_WARN(get_logger(), "no thrusters.table_path set; CMG-only operation");
  }

  attitude_pub_ = create_publisher<geometry_msgs::msg::Quaternion>("/ssos/gnc/attitude_est", 10);
  rate_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/ssos/gnc/angvel_est", 10);
  torque_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/ssos/gnc/torque_cmd", 10);
  torque_cmg_pub_ =
    create_publisher<geometry_msgs::msg::Vector3>("/ssos/gnc/torque_cmg_cmd", 10);
  torque_thr_pub_ =
    create_publisher<geometry_msgs::msg::Vector3>("/ssos/gnc/torque_thr_cmd", 10);
  cmg_momentum_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/ssos/gnc/cmg_momentum", 10);
  gimbal_rate_pub_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/ssos/gnc/cmg_gimbal_rate", 10);
  thruster_pub_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("/ssos/gnc/thruster_duty", 10);
  mode_pub_ = create_publisher<GncModeState>("/ssos/gnc/mode_state", 10);
  diagnostics_pub_ = create_publisher<DiagnosticArray>("/ssos/gnc/diagnostics", 10);
  heartbeat_pub_ = create_publisher<SubsystemHeartbeat>("/ssos/gnc/heartbeat", 10);
  fault_pub_ = create_publisher<FaultEvent>("/ssos/fault_event", 10);

  attitude_sub_ = create_subscription<geometry_msgs::msg::Quaternion>(
    "/ssos/gnc/attitude_meas", 10, std::bind(&AttitudeControlNode::on_attitude, this, _1));
  rate_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "/ssos/gnc/angvel_meas", 10, std::bind(&AttitudeControlNode::on_rate, this, _1));
  gimbal_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    "/ssos/gnc/cmg_gimbal", 10, std::bind(&AttitudeControlNode::on_gimbal, this, _1));
  pose_ref_sub_ = create_subscription<geometry_msgs::msg::Quaternion>(
    "/ssos/gnc/pose_ref", 10, std::bind(&AttitudeControlNode::on_pose_ref, this, _1));
  world_state_sub_ = create_subscription<WorldState>(
    "/sim/world_state", 10, std::bind(&AttitudeControlNode::on_world_state, this, _1));
  unload_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/ssos/gnc/unload_request", 10, std::bind(&AttitudeControlNode::on_unload_request, this, _1));

  set_mode_srv_ = create_service<SetActuationMode>(
    "/ssos/gnc/set_actuation_mode",
    std::bind(&AttitudeControlNode::handle_set_mode, this, _1, _2));
  register_client_ = create_client<RegisterSubsystem>("/ssos/register_subsystem");

  param_cb_handle_ = add_on_set_parameters_callback(
    std::bind(&AttitudeControlNode::on_set_parameters, this, _1));

  RCLCPP_INFO(get_logger(), "configured at %.1f Hz", step_rate_hz_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn AttitudeControlNode::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);

  attitude_pub_->on_activate();
  rate_pub_->on_activate();
  torque_pub_->on_activate();
  torque_cmg_pub_->on_activate();
  torque_thr_pub_->on_activate();
  cmg_momentum_pub_->on_activate();
  gimbal_rate_pub_->on_activate();
  thruster_pub_->on_activate();
  mode_pub_->on_activate();
  diagnostics_pub_->on_activate();
  heartbeat_pub_->on_activate();
  fault_pub_->on_activate();

  system_->reset();
  first_step_ = true;

  const auto period = std::chrono::duration<double>(1.0 / step_rate_hz_);
  step_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&AttitudeControlNode::step, this));

  const auto hb_period = std::chrono::duration<double>(1.0 / std::max(0.1, heartbeat_rate_hz_));
  heartbeat_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(hb_period),
    [this]() {
      heartbeat_pub_->publish(
        build_heartbeat(
          now(), subsystem_name_, SubsystemHeartbeat::LIFECYCLE_ACTIVE,
          last_result_.healthy,
          last_result_.healthy ? "nominal" : "fault active"));
    });

  register_with_manager();
  RCLCPP_INFO(get_logger(), "activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AttitudeControlNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  step_timer_.reset();
  heartbeat_timer_.reset();

  attitude_pub_->on_deactivate();
  rate_pub_->on_deactivate();
  torque_pub_->on_deactivate();
  torque_cmg_pub_->on_deactivate();
  torque_thr_pub_->on_deactivate();
  cmg_momentum_pub_->on_deactivate();
  gimbal_rate_pub_->on_deactivate();
  thruster_pub_->on_deactivate();
  mode_pub_->on_deactivate();
  diagnostics_pub_->on_deactivate();
  heartbeat_pub_->on_deactivate();
  fault_pub_->on_deactivate();

  LifecycleNode::on_deactivate(state);
  return CallbackReturn::SUCCESS;
}

CallbackReturn AttitudeControlNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  heartbeat_timer_.reset();
  system_.reset();
  bridge_.reset();
  loader_.reset();
  return CallbackReturn::SUCCESS;
}

void AttitudeControlNode::on_attitude(const geometry_msgs::msg::Quaternion::SharedPtr msg)
{
  sensors_.star_tracker = common::Quaternion(msg->w, msg->x, msg->y, msg->z);
  sensors_.star_tracker_valid = true;
  have_attitude_ = true;
}

void AttitudeControlNode::on_rate(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  sensors_.gyro = Eigen::Vector3d(msg->x, msg->y, msg->z);
}

void AttitudeControlNode::on_gimbal(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  for (std::size_t i = 0; i < msg->data.size() && i < static_cast<std::size_t>(common::kNumCmg);
    ++i)
  {
    sensors_.gimbal(static_cast<Eigen::Index>(i)) = msg->data[i];
  }
}

void AttitudeControlNode::on_pose_ref(const geometry_msgs::msg::Quaternion::SharedPtr msg)
{
  command_.attitude_ref = common::Quaternion(msg->w, msg->x, msg->y, msg->z);
}

void AttitudeControlNode::on_world_state(const WorldState::SharedPtr)
{
}

void AttitudeControlNode::on_unload_request(const std_msgs::msg::Bool::SharedPtr msg)
{
  system_->set_unload_active(msg->data);
  RCLCPP_INFO(get_logger(), "CMG unloading %s", msg->data ? "requested" : "cancelled");
}

void AttitudeControlNode::handle_set_mode(
  const SetActuationMode::Request::SharedPtr req,
  SetActuationMode::Response::SharedPtr res)
{
  const auto desired = (req->mode == SetActuationMode::Request::MODE_THRUSTER) ?
    flight::ActuationMode::THRUSTER : flight::ActuationMode::CMG;

  const flight::ModeRequestResult r = system_->request_mode(desired);
  res->success = r.accepted;
  res->message = r.reason;
  res->current_mode = (r.current == flight::ActuationMode::THRUSTER) ?
    SetActuationMode::Request::MODE_THRUSTER : SetActuationMode::Request::MODE_CMG;

  RCLCPP_INFO(
    get_logger(), "set_actuation_mode -> %s (%s)",
    flight::ActuationModeMachine::mode_name(r.current), r.reason.c_str());
}

void AttitudeControlNode::step()
{
  if (!system_ || !have_attitude_) {return;}

  const rclcpp::Time current = now();
  double dt = 1.0 / step_rate_hz_;
  if (!first_step_) {
    const double measured = (current - last_step_time_).seconds();
    if (measured > 0.0) {dt = measured;}
  }
  first_step_ = false;
  last_step_time_ = current;

  if (params_dirty_) {
    system_->set_parameters(bridge_->read_flight_parameters());
    params_dirty_ = false;
  }

  last_result_ = system_->step(dt, sensors_, command_);

  publish_outputs(current);
  publish_fault_transitions(current);
}

void AttitudeControlNode::publish_outputs(const rclcpp::Time & stamp)
{
  attitude_pub_->publish(to_quaternion(last_result_.attitude_est));
  rate_pub_->publish(to_vector3(last_result_.rate_est));
  torque_pub_->publish(to_vector3(last_result_.torque_cmd_total));
  torque_cmg_pub_->publish(to_vector3(last_result_.torque_cmg_cmd));
  torque_thr_pub_->publish(to_vector3(last_result_.torque_thr_cmd));
  cmg_momentum_pub_->publish(to_vector3(last_result_.cmg_momentum));

  std_msgs::msg::Float64MultiArray gimbal_rate;
  gimbal_rate.data.assign(
    last_result_.gimbal_rate_cmd.data(),
    last_result_.gimbal_rate_cmd.data() + common::kNumCmg);
  gimbal_rate_pub_->publish(gimbal_rate);

  if (last_result_.thruster_duty.size() > 0) {
    thruster_pub_->publish(to_array(last_result_.thruster_duty));
  }

  GncModeState mode;
  mode.stamp = stamp;
  mode.mode = (last_result_.mode == flight::ActuationMode::THRUSTER) ?
    GncModeState::MODE_THRUSTER : GncModeState::MODE_CMG;
  mode.time_in_mode = system_->mode_machine().time_in_mode();
  mode.transition_count = system_->mode_machine().transition_count();
  mode.forced_by_fault = system_->mode_machine().forced_by_fault();
  mode.unload_active = last_result_.unload_active;
  mode.cmg_momentum_fraction = last_result_.cmg_momentum_frac;
  mode.manipulability = last_result_.manipulability;
  mode_pub_->publish(mode);

  diagnostics_pub_->publish(build_diagnostics(stamp, subsystem_name_, last_result_));
}

void AttitudeControlNode::publish_fault_transitions(const rclcpp::Time & stamp)
{
  for (const auto & fault : system_->fault_injector().active_faults()) {
    (void)fault;
  }
  (void)stamp;
}

void AttitudeControlNode::register_with_manager()
{
  if (!register_client_->wait_for_service(2s)) {
    RCLCPP_WARN(get_logger(), "system_manager not available; continuing unregistered");
    return;
  }

  auto req = std::make_shared<RegisterSubsystem::Request>();
  req->subsystem_name = subsystem_name_;
  req->published_topics = {
    "/ssos/gnc/attitude_est", "/ssos/gnc/angvel_est", "/ssos/gnc/torque_cmd",
    "/ssos/gnc/cmg_momentum", "/ssos/gnc/cmg_gimbal_rate", "/ssos/gnc/thruster_duty",
    "/ssos/gnc/mode_state", "/ssos/gnc/diagnostics",
    "/ssos/gnc/torque_cmg_cmd", "/ssos/gnc/torque_thr_cmd"};
  req->subscribed_topics = {
    "/sim/world_state", "/ssos/gnc/pose_ref", "/ssos/gnc/attitude_meas",
    "/ssos/gnc/angvel_meas", "/ssos/gnc/cmg_gimbal"};
  req->heartbeat_topic = "/ssos/gnc/heartbeat";

  register_client_->async_send_request(req);
  RCLCPP_INFO(get_logger(), "registered with system_manager as '%s'", subsystem_name_.c_str());
}

rcl_interfaces::msg::SetParametersResult AttitudeControlNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  auto result = bridge_->validate(params);
  if (result.successful) {
    params_dirty_ = true;
  }
  return result;
}
}
}
