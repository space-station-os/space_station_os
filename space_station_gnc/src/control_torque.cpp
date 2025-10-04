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

#include "space_station_gnc/action/unloading.hpp"
#include "space_station_gnc/thruster_matrix.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem> // C++17
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "space_station_gnc/srv/set_gnc_mode_actuation.hpp"

static std::string resolve_package_url_to_path(const std::string &uri) {
  const std::string prefix = "package://";
  if (uri.rfind(prefix, 0) == 0) {
    const std::string rest = uri.substr(prefix.size()); // "pkg/path/.."
    const auto slash = rest.find('/');
    if (slash == std::string::npos) {
      throw std::runtime_error("Invalid package URI (no slash): " + uri);
    }
    const std::string pkg = rest.substr(0, slash);
    const std::string rel = rest.substr(slash + 1);
    const std::string share = ament_index_cpp::get_package_share_directory(pkg);
    return share + "/" + rel;
  }
  return uri; // already a filesystem path
}

class ControlTorque : public rclcpp::Node {
public:
  using unloading = space_station_gnc::action::Unloading;
  using GoalHandleUnloading = rclcpp_action::ServerGoalHandle<unloading>;

  using SetModeSrv = space_station_gnc::srv::SetGncModeActuation;

  ControlTorque() : Node("control_torque") {

    // Declare parameters for PD gains
    this->declare_parameter("kp_cmg", 300000.0);
    this->declare_parameter("kd_cmg", 300000.0);
    this->declare_parameter("k_unload", 10.0);
    this->declare_parameter("kp_thruster", 1000000.0);
    this->declare_parameter("kd_thruster", 1000000.0);

    this->declare_parameter("thruster_omega_lpf_tau", 1.0); //
    this->declare_parameter("thruster_angle_lpf_tau", 0.0); //
    this->declare_parameter("lpf_dt", 0.1);                 //

    angvel_filter_timeconst =
        this->get_parameter("thruster_omega_lpf_tau").as_double();
    angpos_filter_timeconst =
        this->get_parameter("thruster_angle_lpf_tau").as_double();
    lpf_dt = this->get_parameter("lpf_dt").as_double();

    auto calc_alpha = [&](double tau) -> double {
      if (tau <= 0.0)
        return 0.0;
      double dt = std::clamp(lpf_dt, 1e-4, 0.2);
      return std::clamp(std::exp(-dt / tau), 0.0, 0.9999);
    };
    angvel_filter_alpha = calc_alpha(angvel_filter_timeconst);
    angpos_filter_alpha = calc_alpha(angpos_filter_timeconst);
    angvel_filter_state.setZero();
    angpos_filter_state.setZero();

    // Subscribers
    pose_ref_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
        "/gnc/pose_ref", rclcpp::QoS(10),
        std::bind(&ControlTorque::callback_pose_ref, this,
                  std::placeholders::_1));

    pose_est_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
        "/gnc/pose_est", rclcpp::QoS(10),
        std::bind(&ControlTorque::callback_pose_est, this,
                  std::placeholders::_1));

    angvel_est_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/gnc/angvel_est", rclcpp::QoS(10),
        std::bind(&ControlTorque::callback_angvel_est, this,
                  std::placeholders::_1));

    cmg_h_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/gnc/cmg_h", rclcpp::QoS(10),
        std::bind(&ControlTorque::callback_cmg_h, this, std::placeholders::_1));

    /*  REMOVED: legacy topic-based mode switching subscriber
    // Reason: To avoid ambiguity and race conditions, switching is now handled
    // exclusively by the synchronous service /gnc/set_mode_actuation.
    control_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/gnc/mode_gnc_control", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
              const std::string &mode = msg->data;
              if (mode == "cmg" || mode == "thruster") {
                mode_gnc_control_ = mode;
                RCLCPP_WARN(this->get_logger(), "Control mode set to: %s",
                            mode_gnc_control_.c_str());
              } else {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Received invalid control mode: '%s'. Keeping current mode:
       %s", mode.c_str(), mode_gnc_control_.c_str());
              }
            });
    */

    urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          if (received_)
            return;
          received_ = true;

          // 1) URDF → Thruster geometry
          thrusterMat.initialize(msg->data);
          thrusterMat.setBaseLink("Root");

          // 2) Table YAML param
          const auto tables_yaml_param = this->declare_parameter<std::string>(
              "thruster_tables_yaml",
              "package://space_station_gnc/config/eagle_thruster_table.yaml");

          const std::string tables_yaml_file =
              resolve_package_url_to_path(tables_yaml_param);
          if (!std::filesystem::exists(tables_yaml_file)) {
            RCLCPP_FATAL(this->get_logger(),
                         "Thruster table file not found: %s",
                         tables_yaml_file.c_str());
            throw std::runtime_error("Thruster table YAML not found");
          }

          // 3) Load & select mode
          thrusterMat.loadThrusterTableFromYaml(tables_yaml_file);

          const auto table_mode =
              this->declare_parameter<std::string>("table_mode", "sixdof_phys");
          thrusterMat.setThrusterTable(table_mode);

          // 4) Load per-thruster properties (limits etc.)
          const auto props_yaml_param = this->declare_parameter<std::string>(
              "thruster_properties_yaml", "package://space_station_gnc/config/"
                                          "eagle_thruster_properties.yaml");
          const std::string props_yaml_file =
              resolve_package_url_to_path(props_yaml_param);
          thrusterMat.loadProperties(props_yaml_file);

          RCLCPP_INFO(this->get_logger(), "Thruster properties loaded: %s",
                      props_yaml_file.c_str());

          // Sanity log
          Eigen::Matrix<double, 6, Eigen::Dynamic> W =
              thrusterMat.buildWForMode(table_mode);
          Eigen::FullPivLU<Eigen::MatrixXd> lu(W);
          int rank = lu.rank();
          double colsum = 0.0;
          for (int i = 0; i < W.cols(); ++i)
            colsum += W.col(i).norm();
          RCLCPP_INFO(this->get_logger(),
                      "Thruster table loaded: %s, path=%s, W: 6x%ld, rank=%d, "
                      "sum|col|=%.3f",
                      table_mode.c_str(), tables_yaml_file.c_str(),
                      static_cast<long>(W.cols()), rank, colsum);

          RCLCPP_INFO(this->get_logger(),
                      "Received robot description and initialised thrusters!!");

          urdf_sub_.reset();
        });

    pose_ref_.x = 0.0;
    pose_ref_.y = 0.0;
    pose_ref_.z = 0.0;
    pose_ref_.w = 1.0;

    // Publisher
    torque_cmg_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "/gnc/torque_cmg_cmd", 10);
    torque_thr_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "/gnc/torque_thr_cmd", 10);
    thr_duty_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/gnc/thr_duty_cmd", 10);
    thr_force_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/gnc/thr_force_cmd", 10);

    // Publisher for current actuation mode (latched) for UI/OpenMCT.
    // Using transient_local so that late-joining subscribers receive the last
    // state.
    mode_actuation_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/gnc/mode_actuation", rclcpp::QoS(1).transient_local());

    // Service for switching actuation mode (replaces topic-based command).
    set_mode_srv_ = this->create_service<SetModeSrv>(
        "/gnc/set_mode_actuation",
        [this](const std::shared_ptr<SetModeSrv::Request> req,
               std::shared_ptr<SetModeSrv::Response> res) {
          const bool ok = this->handle_mode_request(req->mode, "service");
          res->success = ok;
          res->current_mode = mode_gnc_control_;
          res->message = ok ? "Accepted" : "Rejected";
        });

    // Initialize mode and publish initial state
    set_current_mode_and_publish("cmg", "startup");

    // Action server for unloading
    unloading_server_ = rclcpp_action::create_server<unloading>(
        this, "gnc/unloading",
        std::bind(&ControlTorque::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&ControlTorque::handle_cancel, this, std::placeholders::_1),
        std::bind(&ControlTorque::handle_accepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Control Torque Node Initialized");
  }

private:
  // normalize + validate + apply switching + publish current state
  bool handle_mode_request(std::string mode_in, const std::string &source) {
    std::transform(mode_in.begin(), mode_in.end(), mode_in.begin(), ::tolower);

    if (mode_in != "cmg" && mode_in != "thruster" && mode_in != "auto") {
      RCLCPP_WARN(
          this->get_logger(),
          "[%s] invalid actuation mode '%s' (expected: cmg|thruster|auto)",
          source.c_str(), mode_in.c_str());
      return false;
    }

    // TODO: add inhibit checks if needed (e.g., thruster firing blocks CMG
    // switch)
    set_current_mode_and_publish(mode_in, source);
    RCLCPP_INFO(this->get_logger(), "Actuation mode set to: %s (by %s)",
                mode_in.c_str(), source.c_str());
    return true;
  }

  // single source of truth for updating mode + notifying others
  void set_current_mode_and_publish(const std::string &mode,
                                    const std::string &source) {
    mode_gnc_control_ = mode;

    std_msgs::msg::String state;
    state.data = mode_gnc_control_;
    mode_actuation_pub_->publish(state);

    (void)source; // reserved for future provenance logging
  }

  // Publish a 3D zero torque on /gnc/cmg_torque_cmd
  void publish_zero_cmg_torque() {
    geometry_msgs::msg::Vector3 zero;
    zero.x = zero.y = zero.z = 0.0;
    torque_cmg_pub_->publish(zero);
  }

  // Publish a zero-thrust vector of length N on /gnc/thr_duty_cmd
  void publish_zero_thr_torque() {
    geometry_msgs::msg::Vector3 z;
    z.x = z.y = z.z = 0.0;
    torque_thr_pub_->publish(z);
  }

  void publish_zero_thr_duty(std::size_t n) {
    std_msgs::msg::Float32MultiArray zeros;
    zeros.data.assign(n, 0.0f);
    thr_duty_pub_->publish(zeros);
  }

  void callback_pose_ref(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
    pose_ref_ = *msg;
    RCLCPP_INFO(this->get_logger(),
                "Received new pose_ref: [w=%.4f x=%.4f y=%.4f z=%.4f]",
                pose_ref_.w, pose_ref_.x, pose_ref_.y, pose_ref_.z);
  }

  void callback_pose_est(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
    pose_est_ = *msg;

    // Eigen::Quaterniond q_ref_LVLH = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_ref_LVLH =
        Eigen::Quaterniond(pose_ref_.w, pose_ref_.x, pose_ref_.y, pose_ref_.z);
    q_ref_LVLH.normalize();
    Eigen::Quaterniond q_act_LVLH(pose_est_.w, pose_est_.x, pose_est_.y,
                                  pose_est_.z);
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

    // --- attitude diagnostics (small-angle vector already computed upstream)
    // ---
    const double err_ang = error_vector.norm(); // [rad]
    Eigen::Vector3d err_axis = Eigen::Vector3d::Zero();
    if (err_ang > 1e-12)
      err_axis = error_vector / err_ang;

    const double w_norm = error_angvel.norm(); // [rad/s]
    RCLCPP_INFO(this->get_logger(),
                "[att] |err_angle|=%+5.3f deg, axis=[%+.2f %+.2f %+.2f], "
                "omega=%.3f  [%+.3f %+.3f %+.3f] deg/s",
                err_ang * 180.0 / M_PI, err_axis.x(), err_axis.y(),
                err_axis.z(), w_norm * 180.0 / M_PI,
                angvel_est_.x * 180.0 / M_PI, angvel_est_.y * 180.0 / M_PI,
                angvel_est_.z * 180.0 / M_PI);

    if (mode_gnc_control_ == "cmg") {
      compute_cmg_control();
    } else if (mode_gnc_control_ == "thruster") {
      compute_thruster_control();
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown control mode: %s",
                  mode_gnc_control_.c_str());
    }
  }

  void callback_angvel_est(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    angvel_est_ = *msg;
  }
  void callback_cmg_h(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    cmg_pos_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
  }

  void compute_cmg_control() {
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

    torque_cmg_msg.x = torque_cmd.x();
    torque_cmg_msg.y = torque_cmd.y();
    torque_cmg_msg.z = torque_cmd.z();
    torque_cmg_pub_->publish(torque_cmg_msg);

    // Inactive thruster branch: always zero
    const std::size_t n_thr = static_cast<std::size_t>(thrusterMat.getNumThr());
    publish_zero_thr_duty(n_thr);
    publish_zero_thr_torque();
  }

  void compute_thruster_control() {
    // Gains
    const double kp = this->get_parameter("kp_thruster").as_double();
    const double kd = this->get_parameter("kd_thruster").as_double();

    // angpos_filter: e_filt[k] = α*e_filt[k-1] + (1-α)*e_raw[k]
    const Eigen::Vector3d e_raw(error_vector.x(), error_vector.y(),
                                error_vector.z());
    if (angpos_filter_alpha > 0.0) {
      angpos_filter_state = angpos_filter_alpha * angpos_filter_state +
                            (1.0 - angpos_filter_alpha) * e_raw;
    } else {
      angpos_filter_state = e_raw;
    }
    const Eigen::Vector3d w_raw(error_angvel.x(), error_angvel.y(),
                                error_angvel.z());
    if (angvel_filter_alpha > 0.0) {
      angvel_filter_state = angvel_filter_alpha * angvel_filter_state +
                            (1.0 - angvel_filter_alpha) * w_raw;
    } else {
      angvel_filter_state = w_raw;
    }

    // --- Dead-zone with hysteresis using angle AND angular rate ---
    // Units: angle [deg], rate [deg/s]
    static bool dz_active = false;

    // Tunable thresholds (you can move these to ROS parameters if desired)
    const double ang_on_deg =
        0.150; // leave zero region when error grows (upper)
    const double ang_off_deg =
        0.1; // enter zero region when already small (lower)
    const double rate_on_dps =
        0.25; // leave zero when angular rate grows (upper)
    const double rate_off_dps =
        0.15; // enter zero when rate already small (lower)

    // Current values
    const double err_ang_deg = error_vector.norm() * 180.0 / M_PI; // [deg]
    const double rate_dps = error_angvel.norm() * 180.0 / M_PI;    // [deg/s]

    // Enter: BOTH angle <= off AND rate <= off  → quiet enough
    if (!dz_active && (err_ang_deg <= ang_off_deg) &&
        (rate_dps <= rate_off_dps)) {
      dz_active = true;
    } else if (dz_active &&
               (err_ang_deg >= ang_on_deg || rate_dps >= rate_on_dps)) {
      // Leave: EITHER angle >= on OR rate >= on   → motion sizable
      dz_active = false;
    }

    // Build P (or PD) torque command
    Eigen::Vector3d tau_cmd_vec =
        kp * angpos_filter_state + kd * (-angvel_filter_state);

    // Apply dead-zone
    if (dz_active) {
      tau_cmd_vec.setZero();
    }

    // Optional: tiny command clamp even when dz_active==false (avoids micro
    // pulses)
    const double tau_min = 1e-3; // [N·m], tune as needed
    if (tau_cmd_vec.norm() < tau_min) {
      tau_cmd_vec.setZero();
    }

    this->torque_cmd = tau_cmd_vec;

    // Always zero the inactive branch (CMG)
    publish_zero_cmg_torque();

    // Geometry/table readiness
    if (!thrusterMat.isReady()) {
      publish_zero_thr_duty(0);
      publish_zero_thr_torque();
      return;
    }

    // --- Attitude diagnostics (angle/axis and angular-rate) ---
    const double err_ang = error_vector.norm(); // [rad]
    Eigen::Vector3d err_axis = Eigen::Vector3d::Zero();
    if (err_ang > 1e-12)
      err_axis = error_vector / err_ang;
    const double w_norm = error_angvel.norm(); // [rad/s]
    // TODO

    // --- 6DOF table path: build wrench [Tx Ty Tz Fx Fy Fz]^T ---
    Eigen::VectorXd wrench6(6);
    wrench6.setZero();
    wrench6(0) = torque_cmd.x();
    wrench6(1) = torque_cmd.y();
    wrench6(2) = torque_cmd.z();
    // Fx, Fy, Fz are zero for now (translation not commanded here)

    // Solve table allocation: u_thr (force commands per thruster) [N]
    Eigen::VectorXd u_thr;
    const double tol = std::max(1e-8, 1e-6 * torque_cmd.norm());
    try {
      // thrusterMat.generateCommandFromTable(wrench6, u_thr);
      if (!thrusterMat.generateCommandPreferDirect(wrench6, u_thr, tol)) {
        // Safety: if both direct and table failed, zero out
        u_thr = Eigen::VectorXd::Zero(thrusterMat.getNumThr());
      }

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "[diag] generateCommandFromTable exception: %s", e.what());
      publish_zero_thr_duty(0);
      publish_zero_thr_torque();
      return;
    }

    // --- Diagnostics on u_thr before clipping ---
    {
      const double u_norm = (u_thr.size() > 0) ? u_thr.norm() : 0.0;
      int neg_cnt = 0, pos_cnt = 0, zero_cnt = 0;
      for (int i = 0; i < u_thr.size(); ++i) {
        if (u_thr(i) > 1e-12)
          ++pos_cnt;
        else if (u_thr(i) < -1e-12)
          ++neg_cnt;
        else
          ++zero_cnt;
      }
      RCLCPP_DEBUG(
          this->get_logger(),
          "[diag] u_thr.size=%d |u_thr|=%.6f (pos=%d, neg=%d, zero=%d)",
          (int)u_thr.size(), u_norm, pos_cnt, neg_cnt, zero_cnt);
    }

    // --- Load per-thruster limits (already loaded into ThrusterMatrix via
    // loadProperties) --- Convert u_thr -> clamped force vector in [0,
    // max_force] and duty in [0, 1]
    auto active =
        thrusterMat.getActiveThrusters(); // aligned with thruster_order_
    const std::size_t M = std::min<std::size_t>(
        active.size(), static_cast<std::size_t>(u_thr.size()));

    thr_duty_msg.data.resize(static_cast<std::size_t>(u_thr.size()));
    Eigen::VectorXd force =
        Eigen::VectorXd::Zero(u_thr.size()); // [N] used for tau reconstruction

    for (std::size_t i = 0; i < M; ++i) {
      const double fmax = std::max(0.0, active[i].max_force);
      // clamp to [0, fmax] (non-negativity is already applied in table solver)
      const double fi = (fmax > 0.0)
                            ? std::clamp(u_thr(static_cast<int>(i)), 0.0, fmax)
                            : 0.0;
      force(static_cast<int>(i)) = fi;
      const double duty =
          (fmax > 0.0) ? (fi / fmax) : 0.0; // normalized duty [0..1]
      thr_duty_msg.data[i] = static_cast<float>(duty);
    }
    // zero the remainder if any
    for (std::size_t i = M; i < static_cast<std::size_t>(u_thr.size()); ++i)
      thr_duty_msg.data[i] = 0.0f;

    // Publish duty (future PWM layer will quantize if needed)
    thr_duty_pub_->publish(thr_duty_msg);

    // Reconstruct effective torque using *force* (not duty)
    Eigen::Vector3d tau_eff(0.0, 0.0, 0.0);
    try {
      thrusterMat.thrusterToBody(force, tau_eff); // A * force
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "[diag] thrusterToBody exception: %s",
                   e.what());
      publish_zero_thr_torque();
      return;
    }

    // Residuals: table-residual ||W*u_thr - wrench|| and geometric-residual
    // ||A*u_thr - tau_cmd||
    try {
      const std::string mode = this->get_parameter("table_mode").as_string();
      Eigen::Matrix<double, 6, Eigen::Dynamic> W =
          thrusterMat.buildWForMode(mode);
      Eigen::VectorXd wrench_eff = W * u_thr;
      const double resW = (wrench_eff - wrench6).norm();
      const double resA = (tau_eff - torque_cmd).norm();
      RCLCPP_DEBUG(
          this->get_logger(),
          "[diag] residuals: ||W*u_thr - w||=%.6e, ||A*u_thr - tau_cmd||=%.6e",
          resW, resA);
    } catch (...) {
      // ignore diag errors
    }

    // -----------------------------
    // DIAG 1: u_thr (table output)
    // -----------------------------
    {
      // Get names in the same order as allocation columns.
      // Using getActiveThrusters() preserves orderedThrusterNames().size() and
      // order for the active set.
      auto active = thrusterMat.getActiveThrusters();
      const int N = static_cast<int>(u_thr.size());
      RCLCPP_DEBUG(this->get_logger(), "[thr] u_thr (table output) N=%d:", N);
      for (int i = 0; i < N && i < (int)active.size(); ++i) {
        RCLCPP_DEBUG(this->get_logger(), "  %2d  %-16s  u_thr=%.6f", i,
                     active[i].name.c_str(), u_thr(i));
      }

      // Per-column torque contribution from u_thr (using A column i only)
      RCLCPP_DEBUG(this->get_logger(),
                   "[thr] torque contribution from u_thr (A*e_i*u_i):");
      for (int i = 0; i < N; ++i) {
        Eigen::VectorXd ei = Eigen::VectorXd::Zero(N);
        // ei(i) = u_thr(i);
        ei(i) = force(i);
        Eigen::Vector3d tau_i;
        thrusterMat.thrusterToBody(ei, tau_i);
        const char *nm =
            (i < (int)active.size() ? active[i].name.c_str() : "(n/a)");
        RCLCPP_DEBUG(this->get_logger(),
                     "  %2d  %-16s  tau_i=[%.3f %.3f %.3f]  |tau_i|=%.3f", i,
                     nm, tau_i.x(), tau_i.y(), tau_i.z(), tau_i.norm());
      }
    }

    auto print_mat_rowwise = [&](const char *tag,
                                 const Eigen::Ref<const Eigen::MatrixXd> &M,
                                 const std::array<const char *, 3> &rname) {
      for (int r = 0; r < M.rows(); ++r) {
        std::stringstream ss;
        ss.setf(std::ios::fixed);
        ss << std::setprecision(5);
        ss << tag << " " << rname[r] << " [";
        for (int c = 0; c < M.cols(); ++c) {
          ss << std::setw(8) << M(r, c);
          if (c + 1 < M.cols())
            ss << " ";
        }
        ss << "]";
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
      }
    };

    //    RCLCPP_INFO(
    //        this->get_logger(),
    //        "[diag] tau_from_thr = [%6.1f %6.1f %6.1f] |tau_from_thr|=%6.1f  "
    //        "(|tau_cmd|=%6.1f, |force|=%6.1f)",
    //        tau_eff.x(), tau_eff.y(), tau_eff.z(), tau_eff.norm(),
    //        torque_cmd.norm(), force.norm());

    RCLCPP_INFO(this->get_logger(),
                "[diag] |force|=%6.1f,  |tau_cmd|=%7.1f, "
                " |tau_from_thr|=%7.1f | tau_from_thr = [%7.1f %7.1f %7.1f] ",
                force.norm(), torque_cmd.norm(), tau_eff.norm(), tau_eff.x(),
                tau_eff.y(), tau_eff.z());

    auto print_vec_pair = [&](const char *tag,
                              const Eigen::Ref<const Eigen::Vector3d> &a,
                              const Eigen::Ref<const Eigen::Vector3d> &b) {
      RCLCPP_INFO(this->get_logger(),
                  "%s cmd=[%+6.1f %+6.1f %+6.1f]  thr=[%+6.1f %+6.1f %+6.1f]",
                  tag, a.x(), a.y(), a.z(), b.x(), b.y(), b.z());
    };

    // ===== concise diagnostics =====
    {
      const int N = static_cast<int>(u_thr.size());

      // 3xN: per-thruster torque matrix
      Eigen::MatrixXd Tau(3, N);
      Tau.setZero();
      for (int i = 0; i < N; ++i) {
        Eigen::VectorXd ei = Eigen::VectorXd::Zero(N);
        ei(i) = u_thr(i);
        Eigen::Vector3d tau_i;
        thrusterMat.thrusterToBody(ei, tau_i); // A * (e_i * u_i)
        Tau.col(i) = tau_i;
      }

      // tau_cmd と tau_from_thr
      const Eigen::Vector3d tau_cmd_vec = torque_cmd;
      const Eigen::Vector3d tau_thr_vec = tau_eff;

      // 姿勢行列（Body←測定姿勢）
      Eigen::Quaterniond q_act(pose_est_.w, pose_est_.x, pose_est_.y,
                               pose_est_.z);
      q_act.normalize();
      const Eigen::Matrix3d R = q_act.toRotationMatrix();

      // (i) per-thruster torque matrix（3行で出力）
      print_mat_rowwise("[thr/Tau]", Tau, {"Tx:", "Ty:", "Tz:"});

      // (ii) tau_cmd vs tau_from_thr（1行）
      print_vec_pair("[tau]", tau_cmd_vec, tau_thr_vec);

      // (iii) attitude rotation matrix（3行で出力）
      print_mat_rowwise("[att/R]", R, {"r1:", "r2:", "r3:"});
    }

    // Publish torque to the plant
    torque_thr_msg.x = tau_eff.x();
    torque_thr_msg.y = tau_eff.y();
    torque_thr_msg.z = tau_eff.z();
    torque_thr_pub_->publish(torque_thr_msg);

    // (Optional) Per-thruster dump for deeper analysis:
    // auto names = thrusterMat.getAllThrusterNames();
    // for (int i = 0; i < u_thr.size() && i < (int)names.size(); ++i) {
    //   RCLCPP_DEBUG(this->get_logger(), "  %2d  %-16s  u_thr=%.6f", i,
    //   names[i].c_str(), u_thr(i));
    // }
  }

  void handleUnloading(const std::shared_ptr<GoalHandleUnloading> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Handling unloading request");
    rclcpp::Rate rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<unloading::Feedback>();
    auto &status = feedback->rem;
    auto result = std::make_shared<unloading::Result>();
    unload = goal->unload;

    while (rclcpp::ok() && t_bias.norm() > 0.01) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Unloading stopped");
        return;
      }
      status = t_bias.norm(); // TODO: Needs better feedback
      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }

    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Unloading completed");
    }
  }

  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_sub_;
  // default actuation mode & new state publisher / service handles
  std::string mode_gnc_control_ = "cmg"; // default

  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr pose_ref_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr pose_est_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angvel_est_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cmg_h_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  bool received_ = false;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr torque_cmg_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr torque_thr_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr thr_duty_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr thr_force_pub_;
  rclcpp_action::Server<unloading>::SharedPtr unloading_server_;

  // NEW: publish current actuation mode (latched)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_actuation_pub_;
  // NEW: service server for switching actuation mode
  rclcpp::Service<SetModeSrv>::SharedPtr set_mode_srv_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const unloading::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received unloading request, unload: %i",
                goal->unload);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleUnloading> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handle_accepted(const std::shared_ptr<GoalHandleUnloading> goal_handle) {
    std::thread(
        std::bind(&ControlTorque::handleUnloading, this, std::placeholders::_1),
        goal_handle)
        .detach();
  }

  geometry_msgs::msg::Quaternion pose_ref_;
  geometry_msgs::msg::Quaternion pose_est_;
  geometry_msgs::msg::Vector3 angvel_est_;
  Eigen::Vector3d cmg_pos_;

  Eigen::VectorXd thruster_force;

  geometry_msgs::msg::Vector3 torque_cmg_msg;
  geometry_msgs::msg::Vector3 torque_thr_msg;
  std_msgs::msg::Float32MultiArray thr_duty_msg;

  double kp_, kd_, k;
  Eigen::Vector3d k_; // Bias factor for unloading CMG
  Eigen::Vector3d t_bias, t_bias_norm, t_att;
  Eigen::Matrix<double, 3, 3> N;

  Eigen::Vector3d error_vector;
  Eigen::Vector3d error_angvel;
  Eigen::Vector3d torque_cmd;

  // --- LPF states & params (persistent) ---
  Eigen::Vector3d angvel_filter_state{Eigen::Vector3d::Zero()};
  double angvel_filter_timeconst{0.15}; // [s]
  double angvel_filter_alpha{0.0};
  Eigen::Vector3d angpos_filter_state{Eigen::Vector3d::Zero()};
  double angpos_filter_timeconst{0.0}; // [s]
  double angpos_filter_alpha{0.0};
  double lpf_dt{0.1}; // [s]

  std::atomic<bool> unload = false; // Flag to indicate if unloading is required

  ThrusterMatrix thrusterMat;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlTorque>());
  rclcpp::shutdown();
  return 0;
}
