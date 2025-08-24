#include "space_station_gnc/orbit_dynamics.hpp"

using std::placeholders::_1;

OrbitDynamicsNode::OrbitDynamicsNode(const rclcpp::NodeOptions &options)
: Node("orbit_dynamics_node", options)
{
    // ---------- Parameters ----------
    // Timing
    this->declare_parameter<double>("timing.dynamics_dt", 0.25);   // [s]
    this->declare_parameter<double>("timing.publish_dt",  0.5);    // [s]

    // Dynamics
    this->declare_parameter<double>("dynamics.mu", OrbitLib::MU_EARTH);
    this->declare_parameter<double>("dynamics.total_mass", 420000.0); // ~ISS
    this->declare_parameter<bool>("dynamics.use_rk4", false);

    // Topics
    this->declare_parameter<std::string>("topics.bias_thruster_cmd", "gnc/bias_thruster_cmd");
    this->declare_parameter<std::string>("topics.attitude_quat",     "gnc/attitude_body_to_eci");
    this->declare_parameter<std::string>("topics.t_fwd_sim",         "gnc/t_fwd_sim");
    this->declare_parameter<std::string>("topics.pos_eci",           "gnc/pos_eci");
    this->declare_parameter<std::string>("topics.vel_eci",           "gnc/vel_eci");
    this->declare_parameter<std::string>("topics.acc_eci",           "gnc/acc_eci");
    this->declare_parameter<std::string>("topics.pose_eci",          "gnc/pose_eci");
    this->declare_parameter<std::string>("topics.path_eci",          "gnc/path_eci");

    // Viz / frames
    this->declare_parameter<std::string>("frames.eci_frame_id", "ECI");
    this->declare_parameter<int>("viz.path_max_points", 2000);

    // Initialization
    this->declare_parameter<std::string>("initial.tle_line2", "");
    this->declare_parameter<bool>("initial.compute_acc_on_start", true);

    // Read parameters
    dynamics_dt_ = this->get_parameter("timing.dynamics_dt").as_double();
    publish_dt_  = this->get_parameter("timing.publish_dt").as_double();
    mu_          = this->get_parameter("dynamics.mu").as_double();
    mass_        = this->get_parameter("dynamics.total_mass").as_double();
    use_rk4_     = this->get_parameter("dynamics.use_rk4").as_bool();

    topic_thruster_ = this->get_parameter("topics.bias_thruster_cmd").as_string();
    topic_quat_     = this->get_parameter("topics.attitude_quat").as_string();
    topic_t_fwd_    = this->get_parameter("topics.t_fwd_sim").as_string();
    topic_pos_      = this->get_parameter("topics.pos_eci").as_string();
    topic_vel_      = this->get_parameter("topics.vel_eci").as_string();
    topic_acc_      = this->get_parameter("topics.acc_eci").as_string();
    topic_pose_     = this->get_parameter("topics.pose_eci").as_string();
    topic_path_     = this->get_parameter("topics.path_eci").as_string();

    frame_eci_      = this->get_parameter("frames.eci_frame_id").as_string();
    path_max_pts_   = this->get_parameter("viz.path_max_points").as_int();

    // Validate basics
    if (dynamics_dt_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "timing.dynamics_dt <= 0; forcing to 0.1 s");
        dynamics_dt_ = 0.1;
    }
    if (publish_dt_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "timing.publish_dt <= 0; forcing to 0.5 s");
        publish_dt_ = 0.5;
    }
    if (mass_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "dynamics.total_mass <= 0; forcing to 1.0 kg");
        mass_ = 1.0;
    }

    // ---------- Initial state from TLE ----------
    const std::string line2 = this->get_parameter("initial.tle_line2").as_string();
    if (line2.empty()) {
        RCLCPP_ERROR(this->get_logger(), "initial.tle_line2 is empty. Set a valid TLE line 2.");
    } else {
        try {
            OrbitLib::tle_line2_to_eci(line2, r_eci_, v_eci_, mu_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse TLE line 2: %s", e.what());
        }
    }
    if (this->get_parameter("initial.compute_acc_on_start").as_bool()) {
        a_eci_ = calc_acc(r_eci_, Eigen::Vector3d::Zero());
    } else {
        a_eci_.setZero();
    }

    // ---------- Publishers ----------
    pub_pos_  = this->create_publisher<geometry_msgs::msg::Vector3>(topic_pos_, 10);
    pub_vel_  = this->create_publisher<geometry_msgs::msg::Vector3>(topic_vel_, 10);
    pub_acc_  = this->create_publisher<geometry_msgs::msg::Vector3>(topic_acc_, 10);
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_pose_, 10);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>(topic_path_, 10);

    path_msg_.header.frame_id = frame_eci_;

    // ---------- Subscriptions ----------
    sub_thrusters_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        topic_thruster_, 10,
        std::bind(&OrbitDynamicsNode::on_thrusters, this, _1));

    sub_quat_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
        topic_quat_, 10,
        std::bind(&OrbitDynamicsNode::on_quaternion, this, _1));

    sub_t_fwd_ = this->create_subscription<std_msgs::msg::Float64>(
        topic_t_fwd_, 10,
        std::bind(&OrbitDynamicsNode::on_forward_time, this, _1));

    // ---------- Timers ----------
    timer_dynamics_ = this->create_wall_timer(
        std::chrono::duration<double>(dynamics_dt_),
        std::bind(&OrbitDynamicsNode::step_dynamics_once, this));

    timer_publish_ = this->create_wall_timer(
        std::chrono::duration<double>(publish_dt_),
        std::bind(&OrbitDynamicsNode::publish_state, this));

    RCLCPP_INFO(this->get_logger(), "OrbitDynamicsNode ready. dt=%.3f s, pub=%.3f s, RK4=%s",
                dynamics_dt_, publish_dt_, use_rk4_ ? "true" : "false");
}

// ----- Callbacks -----
void OrbitDynamicsNode::on_thrusters(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() != n_thrusters_) {
        RCLCPP_ERROR(this->get_logger(), "Expected %zu thruster inputs, got %zu",
                     n_thrusters_, msg->data.size());
        return;
    }
    for (size_t i = 0; i < n_thrusters_; ++i) {
        const float val = msg->data[i];
        if (std::isnan(val) || std::isinf(val)) {
            RCLCPP_ERROR(this->get_logger(), "Thruster[%zu] invalid value: %f", i, val);
            return;
        }
        thruster_forces_biased_(i) = static_cast<double>(val);
    }
}

void OrbitDynamicsNode::on_quaternion(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
    // ROS msg order: (x,y,z,w)
    q_body_to_eci_ = Eigen::Quaterniond(msg->w, msg->x, msg->y, msg->z).normalized();
}

void OrbitDynamicsNode::on_forward_time(const std_msgs::msg::Float64::SharedPtr msg) {
    double t_forward = msg->data;
    if (t_forward <= 0.0) return;
    if (dynamics_dt_ <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid dynamics_dt_");
        return;
    }
    int steps = static_cast<int>(std::ceil(t_forward / dynamics_dt_));
    steps = std::min(steps, 500000); // safety cap
    for (int i = 0; i < steps; ++i) {
        step_dynamics_once();
        if ((i % 600) == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // avoid starving executor
        }
    }
    
}

// ----- Dynamics core -----
void OrbitDynamicsNode::step_dynamics_once() {
    const double dt = dynamics_dt_;
    const Eigen::Vector3d F_eci = current_thruster_force_eci();

    if (use_rk4_) {
        // State: [r, v]; r' = v; v' = a(r, F)
        const auto ar = [&](const Eigen::Vector3d &r){ return (-mu_ / std::pow(r.norm(), 3)) * r; };

        // k1
        Eigen::Vector3d k1_r = v_eci_;
        Eigen::Vector3d k1_v = ar(r_eci_) + F_eci / mass_;

        // k2
        Eigen::Vector3d r2 = r_eci_ + 0.5*dt*k1_r;
        Eigen::Vector3d v2 = v_eci_ + 0.5*dt*k1_v;
        Eigen::Vector3d k2_r = v2;
        Eigen::Vector3d k2_v = ar(r2) + F_eci / mass_;

        // k3
        Eigen::Vector3d r3 = r_eci_ + 0.5*dt*k2_r;
        Eigen::Vector3d v3 = v_eci_ + 0.5*dt*k2_v;
        Eigen::Vector3d k3_r = v3;
        Eigen::Vector3d k3_v = ar(r3) + F_eci / mass_;

        // k4
        Eigen::Vector3d r4 = r_eci_ + dt*k3_r;
        Eigen::Vector3d v4 = v_eci_ + dt*k3_v;
        Eigen::Vector3d k4_r = v4;
        Eigen::Vector3d k4_v = ar(r4) + F_eci / mass_;

        r_eci_ += (dt/6.0)*(k1_r + 2.0*k2_r + 2.0*k3_r + k4_r);
        v_eci_ += (dt/6.0)*(k1_v + 2.0*k2_v + 2.0*k3_v + k4_v);
        a_eci_  = calc_acc(r_eci_, F_eci);
    } else {
        // Semi‑implicit (symplectic) Euler
        const Eigen::Vector3d a_now = calc_acc(r_eci_, F_eci);
        v_eci_ += a_now * dt;
        r_eci_ += v_eci_ * dt;
        a_eci_  = calc_acc(r_eci_, F_eci);
    }

    append_to_path();
}

void OrbitDynamicsNode::publish_state() {
    // Vector3 publishers
    geometry_msgs::msg::Vector3 v3;
    v3.x = r_eci_.x(); v3.y = r_eci_.y(); v3.z = r_eci_.z();  pub_pos_->publish(v3);
    v3.x = v_eci_.x(); v3.y = v_eci_.y(); v3.z = v_eci_.z();  pub_vel_->publish(v3);
    v3.x = a_eci_.x(); v3.y = a_eci_.y(); v3.z = a_eci_.z();  pub_acc_->publish(v3);

    // PoseStamped (position only in ECI)
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = frame_eci_;
    pose.pose.position.x = r_eci_.x();
    pose.pose.position.y = r_eci_.y();
    pose.pose.position.z = r_eci_.z();
    pose.pose.orientation.w = 1.0; // identity
    pub_pose_->publish(pose);

    // Path
    path_msg_.header.stamp = pose.header.stamp;
    pub_path_->publish(path_msg_);
}

void OrbitDynamicsNode::append_to_path() {
    if (path_max_pts_ <= 0) return;

    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = this->now();
    p.header.frame_id = frame_eci_;
    p.pose.position.x = r_eci_.x();
    p.pose.position.y = r_eci_.y();
    p.pose.position.z = r_eci_.z();
    p.pose.orientation.w = 1.0;

    path_msg_.poses.push_back(p);
    if (static_cast<int>(path_msg_.poses.size()) > path_max_pts_) {
        path_msg_.poses.erase(
            path_msg_.poses.begin(),
            path_msg_.poses.begin() + (path_msg_.poses.size() - path_max_pts_));
    }
}

// ------------ main ------------
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    auto node = std::make_shared<OrbitDynamicsNode>(opts);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
