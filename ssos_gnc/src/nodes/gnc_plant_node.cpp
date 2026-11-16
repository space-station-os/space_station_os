#include "ssos_gnc/nodes/gnc_plant_node.hpp"

#include <chrono>

using std::placeholders::_1;

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
}  // namespace ssos_gnc

GncPlantNode::GncPlantNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("gnc_plant", options)
{
  declare_parameter("subsystem_name", subsystem_name_);
  declare_parameter("step_rate_hz", step_rate_hz_);
  declare_parameter("max_substep_s", 0.02);
  declare_parameter("sensor_seed", 12345);
  declare_parameter("momentum_conserving", false);
}

CallbackReturn GncPlantNode::on_configure(const rclcpp_lifecycle::State &)
{
  subsystem_name_ = get_parameter("subsystem_name").as_string();
  step_rate_hz_ = get_parameter("step_rate_hz").as_double();
  if (step_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "step_rate_hz must be positive");
    return CallbackReturn::FAILURE;
  }

  bridge_ = std::make_unique<GncParameterBridge>(this);
  bridge_->declare_plant_parameters(plant::DisturbanceParams{});

  const plant::Inertia inertia(bridge_->read_inertia_tensor());
  if (!inertia.is_valid()) {
    RCLCPP_ERROR(get_logger(), "inertia tensor is not symmetric positive definite");
    return CallbackReturn::FAILURE;
  }

  body_ = std::make_unique<plant::RigidBody>(inertia);
  body_->set_max_substep(get_parameter("max_substep_s").as_double());
  body_->set_momentum_conserving(get_parameter("momentum_conserving").as_bool());

  disturbances_ =
    std::make_unique<plant::DisturbanceTorques>(bridge_->read_disturbance_parameters());
  sensors_ = std::make_unique<plant::SensorErrorModel>(
    static_cast<std::uint32_t>(get_parameter("sensor_seed").as_int()));

  attitude_pub_ = create_publisher<geometry_msgs::msg::Quaternion>("/ssos/gnc/attitude_meas", 10);
  rate_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/ssos/gnc/angvel_meas", 10);
  gimbal_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/ssos/gnc/cmg_gimbal", 10);
  disturbance_pub_ =
    create_publisher<geometry_msgs::msg::Vector3>("/ssos/gnc/disturbance_torque", 10);
  momentum_pub_ =
    create_publisher<geometry_msgs::msg::Vector3>("/ssos/gnc/total_momentum", 10);
  heartbeat_pub_ = create_publisher<SubsystemHeartbeat>("/ssos/gnc_plant/heartbeat", 10);

  gimbal_rate_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    "/ssos/gnc/cmg_gimbal_rate", 10, std::bind(&GncPlantNode::on_gimbal_rate, this, _1));
  cmg_torque_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "/ssos/gnc/torque_cmg_cmd", 10, std::bind(&GncPlantNode::on_cmg_torque, this, _1));
  thruster_torque_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "/ssos/gnc/torque_thr_cmd", 10, std::bind(&GncPlantNode::on_thruster_torque, this, _1));
  world_state_sub_ = create_subscription<WorldState>(
    "/sim/world_state", 10, std::bind(&GncPlantNode::on_world_state, this, _1));

  RCLCPP_INFO(get_logger(), "plant configured at %.1f Hz", step_rate_hz_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn GncPlantNode::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);

  attitude_pub_->on_activate();
  rate_pub_->on_activate();
  gimbal_pub_->on_activate();
  disturbance_pub_->on_activate();
  momentum_pub_->on_activate();
  heartbeat_pub_->on_activate();

  body_->reset();
  first_step_ = true;

  const auto period = std::chrono::duration<double>(1.0 / step_rate_hz_);
  step_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GncPlantNode::step, this));

  return CallbackReturn::SUCCESS;
}

CallbackReturn GncPlantNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  step_timer_.reset();
  attitude_pub_->on_deactivate();
  rate_pub_->on_deactivate();
  gimbal_pub_->on_deactivate();
  disturbance_pub_->on_deactivate();
  momentum_pub_->on_deactivate();
  heartbeat_pub_->on_deactivate();
  LifecycleNode::on_deactivate(state);
  return CallbackReturn::SUCCESS;
}

CallbackReturn GncPlantNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  step_timer_.reset();
  body_.reset();
  disturbances_.reset();
  sensors_.reset();
  bridge_.reset();
  return CallbackReturn::SUCCESS;
}

void GncPlantNode::on_gimbal_rate(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  for (std::size_t i = 0; i < msg->data.size() && i < static_cast<std::size_t>(common::kNumCmg);
    ++i)
  {
    gimbal_rate_cmd_(static_cast<Eigen::Index>(i)) = msg->data[i];
  }
}

void GncPlantNode::on_cmg_torque(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  cmg_torque_cmd_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
}

void GncPlantNode::on_thruster_torque(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  thruster_torque_cmd_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
}

void GncPlantNode::on_world_state(const WorldState::SharedPtr msg)
{
  env_.altitude_km = msg->altitude_km;
  env_.solar_flux_w_m2 = msg->solar_flux_w_m2;
  env_.in_eclipse = msg->in_eclipse;
}

void GncPlantNode::step()
{
  const rclcpp::Time current = now();
  double dt = 1.0 / step_rate_hz_;
  if (!first_step_) {
    const double measured = (current - last_step_time_).seconds();
    if (measured > 0.0) {dt = measured;}
  }
  first_step_ = false;
  last_step_time_ = current;

  env_.attitude_lvlh = body_->state().attitude;
  const plant::DisturbanceResult dist = disturbances_->compute(env_, body_->inertia());

  plant::RigidBodyInput input;
  input.external_torque = dist.total + thruster_torque_cmd_;
  input.gimbal_rate_cmd = gimbal_rate_cmd_;
  input.cmg_torque_cmd = cmg_torque_cmd_;
  body_->step(dt, input);

  const plant::RigidBodyState & s = body_->state();
  const Eigen::Vector3d measured_rate = sensors_->measure_rate(s.omega, dt);

  common::Quaternion measured_attitude;
  const bool attitude_valid = sensors_->measure_attitude(s.attitude, measured_attitude);

  if (attitude_valid) {
    geometry_msgs::msg::Quaternion q;
    q.x = measured_attitude.x();
    q.y = measured_attitude.y();
    q.z = measured_attitude.z();
    q.w = measured_attitude.w();
    attitude_pub_->publish(q);
  }

  rate_pub_->publish(to_vector3(measured_rate));

  std_msgs::msg::Float64MultiArray gimbal;
  gimbal.data.assign(s.gimbal.data(), s.gimbal.data() + common::kNumCmg);
  gimbal_pub_->publish(gimbal);

  disturbance_pub_->publish(to_vector3(dist.total));
  momentum_pub_->publish(to_vector3(body_->total_momentum()));

  SubsystemHeartbeat hb;
  hb.stamp = current;
  hb.subsystem_name = subsystem_name_;
  hb.lifecycle_state = SubsystemHeartbeat::LIFECYCLE_ACTIVE;
  hb.healthy = !body_->gimbal_rate_saturated();
  hb.status_message = body_->gimbal_rate_saturated() ?
    "gimbal rate saturated" : "nominal";
  heartbeat_pub_->publish(hb);
}
}
}
