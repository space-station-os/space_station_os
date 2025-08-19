#pragma once

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <iostream>
#include <string>
#include <cmath>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <vector>
#include "space_station_gnc/thruster_matrix.hpp"

// =================== Math helpers (header-inline) ===================
namespace Math {
    constexpr double PI = 3.141592653589793;
    constexpr double TWO_PI = 2.0 * PI;
    inline constexpr double deg2rad(double deg) { return deg * PI / 180.0; }
    inline constexpr double rad2deg(double rad) { return rad * 180.0 / PI; }
}

// =================== Orbit utilities (header-inline) ===================
namespace OrbitLib {
    // Earth GM [m^3/s^2]
    constexpr double MU_EARTH = 3.986004418e14;

    // WGS‑84 (kept for potential ECI→ECEF→LLA use)
    inline constexpr double WGS84_A = 6378137.0;
    inline constexpr double WGS84_F = 1.0 / 298.257223563;
    inline constexpr double WGS84_E2 = WGS84_F * (2.0 - WGS84_F);

    // Convert mean motion (rad/s) to semi‑major axis [m]
    inline double mean_motion_to_a(double n) { // n: rad/s
        return std::pow(MU_EARTH / (n * n), 1.0/3.0);
    }

    // Mean anomaly (M) → true anomaly (ν) via Newton iterations
    inline double mean_to_true_anomaly(double M, double e, double tol = 1e-10) {
        M = std::fmod(M, Math::TWO_PI);
        if (M < 0) M += Math::TWO_PI;
        double E = (e < 0.8) ? M : Math::PI;
        for (int i = 0; i < 50; ++i) {
            const double f  = E - e * std::sin(E) - M;
            const double df = 1.0 - e * std::cos(E);
            const double dE = f / df;
            E -= dE;
            if (std::abs(dE) < tol) break;
        }
        const double cosE = std::cos(E);
        const double sinE = std::sin(E);
        return std::atan2(std::sqrt(1.0 - e*e) * sinE, cosE - e);
    }

    // Parse TLE line 2 for elements
    inline void extract_from_tle_line2(
        const std::string &line2,
        double &mean_motion_rev_per_day,
        double &eccentricity,
        double &incl_deg,
        double &raan_deg,
        double &argp_deg,
        double &mean_anom_deg)
    {
        // Expecting standard TLE line 2 formatting
        incl_deg                   = std::stod(line2.substr(8, 8));
        raan_deg                   = std::stod(line2.substr(17, 8));
        eccentricity               = std::stod(std::string("0.") + line2.substr(26, 7));
        argp_deg                   = std::stod(line2.substr(34, 8));
        mean_anom_deg              = std::stod(line2.substr(43, 8));
        mean_motion_rev_per_day    = std::stod(line2.substr(52, 11));
    }

    // Keplerian → ECI state
    inline void kepler_to_eci(
        double n_rev_per_day,   // Mean motion [rev/day]
        double e,               // eccentricity
        double i_deg,           // inclination [deg]
        double raan_deg,        // RAAN [deg]
        double argp_deg,        // argument of perigee [deg]
        double M_deg,           // mean anomaly [deg]
        Eigen::Vector3d &r_eci, // out: position [m]
        Eigen::Vector3d &v_eci, // out: velocity [m/s]
        double mu = MU_EARTH)
    {
        const double n    = n_rev_per_day * Math::TWO_PI / 86400.0; // rad/s
        const double a    = mean_motion_to_a(n);
        const double i    = Math::deg2rad(i_deg);
        const double raan = Math::deg2rad(raan_deg);
        const double argp = Math::deg2rad(argp_deg);
        const double M    = Math::deg2rad(M_deg);

        const double nu = mean_to_true_anomaly(M, e);
        const double p  = a * (1.0 - e*e);
        const double r  = p / (1.0 + e * std::cos(nu));

        const double x_p  = r * std::cos(nu);
        const double y_p  = r * std::sin(nu);
        const double vx_p = -std::sqrt(mu / p) * std::sin(nu);
        const double vy_p =  std::sqrt(mu / p) * (e + std::cos(nu));

        const Eigen::Matrix3d R =
            Eigen::AngleAxisd(raan, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
            Eigen::AngleAxisd(i,    Eigen::Vector3d::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(argp, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        r_eci = R * Eigen::Vector3d(x_p,  y_p,  0.0);
        v_eci = R * Eigen::Vector3d(vx_p, vy_p, 0.0);
    }

    // TLE line 2 → ECI state
    inline void tle_line2_to_eci(
        const std::string &line2,
        Eigen::Vector3d &r_eci,
        Eigen::Vector3d &v_eci,
        double mu = MU_EARTH)
    {
        double n_rev_day, e, incl, raan, argp, M;
        extract_from_tle_line2(line2, n_rev_day, e, incl, raan, argp, M);
        kepler_to_eci(n_rev_day, e, incl, raan, argp, M, r_eci, v_eci, mu);
    }
}

// =================== OrbitDynamicsNode ===================
class OrbitDynamicsNode : public rclcpp::Node {
public:
    explicit OrbitDynamicsNode(const rclcpp::NodeOptions &options);

private:
    // ----- Parameters / settings -----
    double dynamics_dt_{0.25};
    double publish_dt_{0.5};
    double mu_{OrbitLib::MU_EARTH};
    double mass_{420000.0};
    bool   use_rk4_{false};

    std::string topic_thruster_, topic_quat_, topic_t_fwd_;
    std::string topic_pos_, topic_vel_, topic_acc_, topic_pose_, topic_path_;
    std::string frame_eci_;
    int path_max_pts_{2000};

    // ----- State -----
    // Elapsed time [s]
    double sim_t{0.0};
    const size_t n_thrusters_{12};
    Eigen::VectorXd thruster_forces_biased_{Eigen::VectorXd::Zero(12)}; // [N]
    Eigen::Vector3d r_eci_{Eigen::Vector3d::Zero()}; // [m]
    Eigen::Vector3d v_eci_{Eigen::Vector3d::Zero()}; // [m/s]
    Eigen::Vector3d a_eci_{Eigen::Vector3d::Zero()}; // [m/s^2]
    Eigen::Quaterniond q_body_to_eci_{1.0, 0.0, 0.0, 0.0}; // w,x,y,z

    ThrusterMatrix thruster_matrix_; // maps 12×1 → body force (3×1)

    // ----- ROS I/O -----
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_thrusters_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr   sub_quat_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr           sub_t_fwd_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr         pub_pos_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr         pub_vel_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr         pub_acc_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     pub_pose_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                 pub_path_;

    nav_msgs::msg::Path path_msg_;

    rclcpp::TimerBase::SharedPtr timer_dynamics_;
    rclcpp::TimerBase::SharedPtr timer_publish_;

    // ----- Helpers (header-inline per your request) -----
    inline Eigen::Vector3d calc_acc(const Eigen::Vector3d &r_eci, const Eigen::Vector3d &F_eci) const {
        const double r = r_eci.norm();
        Eigen::Vector3d a_g = Eigen::Vector3d::Zero();
        if (r > 1.0) {
            a_g = (-mu_ / (r*r*r)) * r_eci;
        }
        const Eigen::Vector3d a_t = F_eci / mass_;
        return a_g + a_t;
    }

    inline Eigen::Vector3d current_thruster_force_eci() const {
        Eigen::Vector3d F_body;
        thruster_matrix_.thrusterToBody(thruster_forces_biased_, F_body);
        Eigen::Quaterniond q_norm = q_body_to_eci_.normalized();
        return q_norm * F_body; // rotate BF → ECI
    }

    // ----- Callbacks / core -----
    void on_thrusters(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void on_quaternion(const geometry_msgs::msg::Quaternion::SharedPtr msg);
    void on_forward_time(const std_msgs::msg::Float64::SharedPtr msg);

    void step_dynamics_once();
    void publish_state();
    void append_to_path();
};

// main is kept in the .cpp
