// Copyright 2025 Space Station OS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include "space_station_gnc/action/unloading.hpp"
#include "space_station_gnc/thruster_matrix.hpp"

class ControlTorque : public rclcpp::Node
{
public:
  using unloading = space_station_gnc::action::Unloading;
  using GoalHandleUnloading = rclcpp_action::ServerGoalHandle<unloading>;
  ControlTorque()
  : Node("control_torque")
  {

    // Declare parameters for PD gains
    this->declare_parameter("kp_cmg", 300000.0);
    this->declare_parameter("kd_cmg", 300000.0);
    this->declare_parameter("k_unload", 10.0);
    this->declare_parameter("kp_thruster", 10000.0);
    this->declare_parameter("kd_thruster", 8000.0);

    // Subscribers
    pose_ref_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
      "/gnc/pose_ref", rclcpp::QoS(10),
      std::bind(&ControlTorque::callback_pose_ref, this, std::placeholders::_1));

    pose_est_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
      "/gnc/pose_est", rclcpp::QoS(10),
      std::bind(&ControlTorque::callback_pose_est, this, std::placeholders::_1));

    angvel_est_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/gnc/angvel_est", rclcpp::QoS(10),
      std::bind(&ControlTorque::callback_angvel_est, this, std::placeholders::_1));

    cmg_h_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/gnc/cmg_h", rclcpp::QoS(10),
      std::bind(&ControlTorque::callback_cmg_h, this, std::placeholders::_1));

    control_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/gnc/mode_gnc_control", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        const std::string & mode = msg->data;
        if (mode == "cmg" || mode == "thruster") {
          mode_gnc_control_ = mode;
          RCLCPP_INFO(this->get_logger(), "Control mode set to: %s", mode_gnc_control_.c_str());
        } else {
          RCLCPP_WARN(
            this->get_logger(), "Received invalid control mode: '%s'. Keeping current mode: %s",
            mode.c_str(), mode_gnc_control_.c_str());
        }
      });

    urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_description", 10,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        if (!received_) {
          received_ = true;
          thrusterMat.initialize(msg->data);
          RCLCPP_INFO(this->get_logger(), "Received robot description and initialised thrusters!!");

          urdf_sub_.reset();

        }

      });

    pose_ref_.x = 0.0;
    pose_ref_.y = 0.0;
    pose_ref_.z = 0.0;
    pose_ref_.w = 1.0;


    // Publisher
    torque_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/gnc/cmg_torque_cmd", 10);
    torque_thr_pub_ =
      this->create_publisher<geometry_msgs::msg::Vector3>("/gnc/bias_torque_cmd", 10);
    ind_thr_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/gnc/bias_thruster_cmd", 10);

    // Action server for unloading
    unloading_server_ = rclcpp_action::create_server<unloading>(
      this,
      "gnc/unloading",
      std::bind(&ControlTorque::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ControlTorque::handle_cancel, this, std::placeholders::_1),
      std::bind(&ControlTorque::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Control Torque Node Initialized");
  }

private:
  void callback_pose_ref(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    pose_ref_ = *msg;
  }

  void callback_pose_est(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    pose_est_ = *msg;

    Eigen::Quaterniond q_ref_LVLH = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_act_LVLH(pose_est_.w, pose_est_.x, pose_est_.y, pose_est_.z);
    q_act_LVLH.normalize();

    Eigen::Quaterniond q_error = q_act_LVLH.conjugate() * q_ref_LVLH;
    if (q_error.w() < 0) {
      q_error.coeffs() *= -1;
    }

    // Numerically safer angle/axis extraction
    double w = std::clamp(q_error.w(), -1.0, 1.0);
    double sin_half_theta = std::sqrt(std::max(0.0, 1.0 - w * w));

    // atan2 is more robust for angles near 0/π
    double theta = 2.0 * std::atan2(sin_half_theta, w);

    Eigen::Vector3d error_axis_norm;
    if (sin_half_theta > 1e-12) {
      error_axis_norm = q_error.vec() / sin_half_theta;
    } else {
      error_axis_norm.setZero();
    }

    error_vector = theta * error_axis_norm;
    error_angvel = Eigen::Vector3d(angvel_est_.x, angvel_est_.y, angvel_est_.z);


    if (mode_gnc_control_ == "cmg") {
      compute_cmg_control();
    } else if (mode_gnc_control_ == "thruster") {
      compute_thruster_control();
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown control mode: %s", mode_gnc_control_.c_str());
    }
  }

  void callback_angvel_est(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    angvel_est_ = *msg;
  }
  void callback_cmg_h(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    cmg_pos_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
  }

  void compute_cmg_control()
  {
    double kp = this->get_parameter("kp_cmg").as_double();
    double kd = this->get_parameter("kd_cmg").as_double();


    this->torque_cmd = kp * error_vector + kd * (-error_angvel);
    Eigen::Vector3d torque_thr_cmd = Eigen::Vector3d::Zero();

    if (unload) {
      double k = this->get_parameter("k_unload").as_double();
      const Eigen::Vector3d unload_gain_ = Eigen::Vector3d::Constant(k);
      t_bias = (-1) * unload_gain_.asDiagonal() * cmg_pos_;

      const double nrm = t_bias.norm();
      if (nrm > 1e-12) {
        t_bias_norm = t_bias.normalized();

        N = Eigen::Matrix3d::Identity() - t_bias_norm * t_bias_norm.transpose();
        t_att = N * torque_cmd;
        torque_thr_cmd = t_bias + (torque_cmd - t_att);
        torque_cmd = t_att;
      } else {
        // If bias is (near) zero, skip projection to avoid NaNs
        torque_thr_cmd.setZero();
      }
    }

    torque_msg.x = torque_cmd.x();
    torque_msg.y = torque_cmd.y();
    torque_msg.z = torque_cmd.z();
    torque_pub_->publish(torque_msg);
  }

  void compute_thruster_control()
  {
    double kp = this->get_parameter("kp_thruster").as_double();
    double kd = this->get_parameter("kd_thruster").as_double();

    this->torque_cmd = kp * error_vector + kd * (-error_angvel);

    if (!thrusterMat.isReady()) {
      return;
    }

    thrusterMat.bodyToThruster(torque_cmd, thruster_force);
    ind_thr_msg.data.resize(thrusterMat.getNumThr());
    size_t idx = 0;
    for (auto & val : ind_thr_msg.data) {
      val = thruster_force(idx);
      if ((unsigned int)idx != (unsigned int)thruster_force.size()) {
        idx++;
      }
    }
    ind_thr_pub_->publish(ind_thr_msg);
  }


  void handleUnloading(const std::shared_ptr<GoalHandleUnloading> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Handling unloading request");
    rclcpp::Rate rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<unloading::Feedback>();
    auto & status = feedback->rem;
    auto result = std::make_shared<unloading::Result>();
    unload = goal->unload;

    while (rclcpp::ok() && t_bias.norm() > 0.01) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Unloading stopped");
        return;
      }
      status = t_bias.norm();       // TODO: Needs better feedback
      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }

    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Unloading completed");
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_sub_;
  std::string mode_gnc_control_ = "cmg";   // default

  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr pose_ref_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr pose_est_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angvel_est_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cmg_h_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  bool received_ = false;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr torque_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr torque_thr_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ind_thr_pub_;
  rclcpp_action::Server<unloading>::SharedPtr unloading_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const unloading::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received unloading request, unload: %i", goal->unload);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleUnloading> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handle_accepted(const std::shared_ptr<GoalHandleUnloading> goal_handle)
  {
    std::thread(
      std::bind(&ControlTorque::handleUnloading, this, std::placeholders::_1),
      goal_handle).detach();
  }

  geometry_msgs::msg::Quaternion pose_ref_;
  geometry_msgs::msg::Quaternion pose_est_;
  geometry_msgs::msg::Vector3 angvel_est_;
  Eigen::Vector3d cmg_pos_;

  Eigen::VectorXd thruster_force;

  geometry_msgs::msg::Vector3 torque_msg;
  geometry_msgs::msg::Vector3 torque_thr_msg;
  std_msgs::msg::Float32MultiArray ind_thr_msg;

  double kp_, kd_, k;
  Eigen::Vector3d k_;   // Bias factor for unloading CMG
  Eigen::Vector3d t_bias, t_bias_norm, t_att;
  Eigen::Matrix<double, 3, 3> N;

  Eigen::Vector3d error_vector;
  Eigen::Vector3d error_angvel;
  Eigen::Vector3d torque_cmd;

  std::atomic<bool> unload = false;   // Flag to indicate if unloading is required

  ThrusterMatrix thrusterMat;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlTorque>());
  rclcpp::shutdown();
  return 0;
}
