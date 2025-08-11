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
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <vector>
#include "L_p_func.cpp"
#include "space_station_gnc/action/unloading.hpp"
#include "space_station_gnc/thruster_matrix.hpp"
//std::string mode_demo;


namespace Math {
    constexpr double PI = 3.141592653589793;
    constexpr double TWO_PI = 2.0 * PI;

    inline constexpr double deg2rad(double deg) { return deg / 180.0 * Math::PI; }
    inline constexpr double rad2deg(double rad) { return rad / Math::PI * 180.0; }

}


namespace OrbitLib {

    // The Earth radius [m]
    constexpr double EARTH_RADIUS = 6378.14 * 1e3;
    // [m^3 s^-2]
    constexpr double G_ME = 3.986004418e14;

    // Ditstance from the Sun to the Earth [m]
    constexpr double SUN_EARTH_DISTANCE = 149600000 * 1e3;

    // J2
    constexpr double J2 = 1.08263e-3;

    // Earth Constant (WGS84)
    // major axis [m]
    const double WGS84_A = 6378137.0;
    // flattening
    const double WGS84_F = 1.0 / 298.257223563;
    // square of eccentricity
    const double WGS84_E2 = WGS84_F * (2 - WGS84_F);
    // Earth rotation angular velocity [rad/s]
    const double EARTH_OMEGA = 7.2921150e-5;

    // ======== Mean motion [rad/s] -> semi-major axix [m] ========
    double mean_motion_to_a(double n) {
        return std::pow(G_ME / (n * n), 1.0 / 3.0);
    }

    // ======== Calculate true anomaly by solving kepler equation ========
    double mean_to_true_anomaly(double M, double e, double tol = 1e-8) {
        M = std::fmod(M, Math::TWO_PI);
        double E = M;
        for (int i = 0; i < 100; ++i) {
            double f = E - e * sin(E) - M;
            double df = 1 - e * cos(E);
            double dE = f / df;
            E -= dE;
            if (fabs(dE) < tol) break;
        }
        double cosE = cos(E);
        double sinE = sin(E);
        double true_anom = atan2(sqrt(1 - e * e) * sinE, cosE - e);
        return true_anom;
    }

    // ======== Extract kepler elements from line 2 of TLE ========
    void extract_element_from_tle_line2(
        std::string line2,
        double& mean_motion_rev_per_day,
        double& eccentricity_no_decimal,
        double& inclination_deg,
        double& raan_deg,
        double& argp_deg,
        double& mean_anomaly_deg
    ) {
        inclination_deg = std::stod(line2.substr(8, 8));
        raan_deg = std::stod(line2.substr(17, 8));
        eccentricity_no_decimal = std::stod("0." + line2.substr(26, 7));
        argp_deg = std::stod(line2.substr(34, 8));
        mean_anomaly_deg = std::stod(line2.substr(43, 8));
        mean_motion_rev_per_day = std::stod(line2.substr(52, 11));
    }

    // ======== Calculate ECI position and velocity from TLE elements ========
    void convert_tle_element_to_eci(
        double mean_motion_rev_per_day,
        double eccentricity_no_decimal,
        double inclination_deg,
        double raan_deg,
        double argp_deg,
        double mean_anomaly_deg,
        Eigen::Vector3d& position,
        Eigen::Vector3d& velocity
    ) {
        double n = mean_motion_rev_per_day * Math::TWO_PI / 86400.0; // rad/s
        double a = mean_motion_to_a(n); // [m]
        double e = eccentricity_no_decimal;
        double i = Math::deg2rad(inclination_deg);
        double raan = Math::deg2rad(raan_deg);
        double argp = Math::deg2rad(argp_deg);
        double M = Math::deg2rad(mean_anomaly_deg);
        double nu = mean_to_true_anomaly(M, e);

        // Other related values
        double p = a * (1 - e * e);
        double r = p / (1 + e * cos(nu));
        double x_p = r * cos(nu);
        double y_p = r * sin(nu);

        double vx_p = -sqrt(OrbitLib::G_ME / p) * sin(nu);
        double vy_p = sqrt(OrbitLib::G_ME / p) * (e + cos(nu));

        // Rotation matrix (Z-X-Z rotation)
        Eigen::Matrix3d r_mat =
            Eigen::AngleAxisd(raan, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
            Eigen::AngleAxisd(i, Eigen::Vector3d::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(argp, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // Tranclate position and velocity into ECI
        Eigen::Vector3d r_p(x_p, y_p, 0.0);
        Eigen::Vector3d v_p(vx_p, vy_p, 0.0);
        position = r_mat * r_p;
        velocity = r_mat * v_p;
    }


    void convert_tle_to_eci(
        std::string tle_line2, 
        Eigen::Vector3d& ss_position_eci, Eigen::Vector3d& ss_velocity_eci
    ) {

        double mean_motion_rev_per_day;
        double eccentricity_no_decimal;
        double inclination_deg;
        double raan_deg;
        double argp_deg;
        double mean_anomaly_deg;

        // Extract TLE element
        OrbitLib::extract_element_from_tle_line2(
            tle_line2,
            mean_motion_rev_per_day,
            eccentricity_no_decimal,
            inclination_deg,
            raan_deg,
            argp_deg,
            mean_anomaly_deg
        );

        OrbitLib::convert_tle_element_to_eci(
            mean_motion_rev_per_day,
            eccentricity_no_decimal,
            inclination_deg,
            raan_deg,
            argp_deg,
            mean_anomaly_deg,
            ss_position_eci, ss_velocity_eci
        );

    }

    // ======== UTC -> GMST (simple. Error is small if a several days.) ========
    double unix_time_to_gmst(int64_t unix_seconds, int microseconds) {
        // Seconds since UNIX epoch (1970-01-01T00:00:00 UTC) to J2000 starting point
        // Julian Day of Unix Epoch
        const double JD_UNIX_EPOCH = 2440587.5;   
        // Julian Day of J2000 Epoch
        const double JD_J2000 = 2451545.0;        
        double dt_days = (double)(unix_seconds) / 86400.0 + (double)(microseconds) / (86400.0 * 1e6);
        double jd = JD_UNIX_EPOCH + dt_days;
        double d = jd - JD_J2000;

        // GMST in seconds
        double gmst_sec = 67310.54841 + (876600.0 * 3600.0 + 8640184.812866) * d / 36525.0
            + 0.093104 * (d / 36525.0) * (d / 36525.0)
            - 6.2e-6 * (d / 36525.0) * (d / 36525.0) * (d / 36525.0);

        gmst_sec = fmod(gmst_sec, 86400.0);
        if (gmst_sec < 0) gmst_sec += 86400.0;

        double gmst_rad = Math::TWO_PI * (gmst_sec / 86400.0);
        return gmst_rad; // [rad]
    }

    void eci_to_geodetic(
        const Eigen::Vector3d& position_eci_m,
        int64_t unix_seconds,
        int microseconds,
        double& latitude_deg,
        double& longitude_deg,
        double& altitude_m
    ) {
        // Step1: ECI -> ECEF rotation
        double gmst = unix_time_to_gmst(unix_seconds, microseconds);
        Eigen::Matrix3d R;
        R <<
            cos(gmst), sin(gmst), 0,
            -sin(gmst), cos(gmst), 0,
            0, 0, 1;
        Eigen::Vector3d position_ecef_m = R * position_eci_m;

        double x = position_ecef_m.x();
        double y = position_ecef_m.y();
        double z = position_ecef_m.z();

        // Step2: ECEF -> lat & lon & alt
        double r = sqrt(x * x + y * y);
        double lon = atan2(y, x);

        // Set tempolary phis as initial value
        double lat = atan2(z, r);
        double alt = 0.0;

        double N = 0.0;
        double lat_prev = 0.0;
        // Convergence condition
        const double tol = 1e-8; 

        for (int iter = 0; iter < 10; ++iter) {
            lat_prev = lat;
            N = WGS84_A / sqrt(1 - WGS84_E2 * sin(lat) * sin(lat));
            alt = r / cos(lat) - N;
            lat = atan2(z, r * (1 - WGS84_E2 * (N / (N + alt))));
            if (fabs(lat - lat_prev) < tol) {
                break;
            }
        }

        latitude_deg = Math::rad2deg(lat);
        longitude_deg = Math::rad2deg(lon);
        altitude_m = alt;
    }
}


Eigen::Matrix3d quat2dcm(const Eigen::Vector4d& quat_vec) {

    const Eigen::Vector4d& q = quat_vec;
    auto qs = quat_vec.array() * quat_vec.array();

    Eigen::Matrix3d dcm_mat;

    dcm_mat.coeffRef(0, 0) = qs(0) - qs(1) - qs(2) + qs(3);
    dcm_mat.coeffRef(0, 1) = 2 * (q(0) * q(1) + q(2) * q(3));
    dcm_mat.coeffRef(0, 2) = 2 * (q(0) * q(2) - q(1) * q(3));

    dcm_mat.coeffRef(1, 0) = 2 * (q(0) * q(1) - q(2) * q(3));
    dcm_mat.coeffRef(1, 1) = qs(1) - qs(0) - qs(2) + qs(3);
    dcm_mat.coeffRef(1, 2) = 2 * (q(1) * q(2) + q(0) * q(3));

    dcm_mat.coeffRef(2, 0) = 2 * (q(0) * q(2) + q(1) * q(3));
    dcm_mat.coeffRef(2, 1) = 2 * (q(1) * q(2) - q(0) * q(3));
    dcm_mat.coeffRef(2, 2) = qs(2) - qs(0) - qs(1) + qs(3);

    return dcm_mat;
}


class OrbitDynamicsNode : public rclcpp::Node
{
public:
    using unloading =  space_station_gnc::action::Unloading;
    using GoalHandleUnloading = rclcpp_action::ClientGoalHandle<unloading>;
    OrbitDynamicsNode(const rclcpp::NodeOptions & options) : Node("orbit_physics_motion", options)
    {
        // -------- Iintial parameters --------
        this->Ttorque_ = this->get_parameter("timing.torque_dt").as_double();
        this->Tpubatt_ = this->get_parameter("timing.pub_dt").as_double();
        this->N2disp   = this->get_parameter("timing.publish_every").as_int();

        std::string tle_line2;
        this->get_parameter("initial.tle_line2", tle_line2);

        OrbitLib::convert_tle_to_eci(tle_line2, this->pos_eci_cur, this->vel_eci_cur);

        this->acc_eci_cur = this->calc_acc(this->pos_eci_cur, Eigen::Vector3d::Zero());

        this->pub_pos_eci = this->create_publisher<geometry_msgs::msg::Vector3>("gnc/pos_eci", 10);
        // this->pub_vel_eci = this->create_publisher<geometry_msgs::msg::Vector3>("gnc/vel_eci", 10);
        // this->pub_acc_eci = this->create_publisher<geometry_msgs::msg::Vector3>("gnc/acc_eci", 10);
        
        this->i2disp = 0; //publish rate: Npublish * Ttorque

        // ---- Dynamics Parameter ----
        
        //TODO: handled by parameter surver?
        this->total_mass = this->get_parameter("dynamics.total_mass").as_double();
        mu = this->get_parameter("dynamics.mu").as_double();

        // ---- Subscription ----

        this->sub_t_fwd_sim = this->create_subscription<std_msgs::msg::Float64>(
            "gnc/t_fwd_sim", 1, 
            std::bind(&OrbitDynamicsNode::callback_t_fwd_sim, this, std::placeholders::_1));

        this->sub_torque_control = this->create_subscription<geometry_msgs::msg::Vector3>(
            "gnc/thr_torque_cmd", 1, 
            std::bind(&OrbitDynamicsNode::callback_orbit_dynamics, this, std::placeholders::_1));
        
        this->sub_bias_thruster_control = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "gnc/bias_thruster_cmd", 1, 
            std::bind(&OrbitDynamicsNode::callback_bias_thruster_inp, this, std::placeholders::_1));
        
        this->sub_attitude_quat = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "gnc/attitude_LVLH", 1, 
            std::bind(&OrbitDynamicsNode::callback_attitude_quat, this, std::placeholders::_1));
        
        // ---- Publish timer ----

        timer_pos_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(Tpubatt_*1000.0)), 
            std::bind(&OrbitDynamicsNode::callback_timer_pub_pos, this));

        // ---- Initialize variables ----
        this->bias_thruster_input = Eigen::VectorXd(this->n_thruster);
    }

private:

    // -------- Variables --------
    // The number of the thrusters
    const size_t n_thruster = 12;
    
    // ---- Subscription ----
    // 
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_t_fwd_sim;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_bias_thruster_control;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_torque_control;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_attitude_quat;

    // ---- Publisher ----
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_pos_eci;
    // rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_vel_eci;
    // rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_acc_eci;

    rclcpp::TimerBase::SharedPtr timer_pos_;

    // ---- Parameters for dynamics ----
    // Gravitational parameter (m^3/s^2)
    double mu;
    // Total mass [kg]
    double total_mass;

    // Thruster model
    ThrusterMatrix thrusterMat;
    
    // Position, velocity and acceleration of space station at Earth Centered Inertial Frame (ECI) [m], [m/s], [m/s^2]
    Eigen::Vector3d pos_eci_cur;
    Eigen::Vector3d vel_eci_cur;
    Eigen::Vector3d acc_eci_cur;

    // thruster force [N]
    Eigen::VectorXd bias_thruster_input;

    // Attitude Quaternion
    Eigen::Vector4d attitude_quat;

    // ---- Define parameters for simulation ----
    //same as T_callback
    double Ttorque_; 

    // publish rate: Npublish * Ttorque
    int N2disp; 
    // index for i2disp
    int i2disp; 
    double Tpubatt_; 

    // -------- Callback functions --------

    void callback_timer_pub_pos() {

        geometry_msgs::msg::Vector3 pos_eci_msg;
        pos_eci_msg.x = this->pos_eci_cur[0];
        pos_eci_msg.y = this->vel_eci_cur[1];
        pos_eci_msg.z = this->acc_eci_cur[2];
        this->pub_pos_eci->publish(pos_eci_msg);
    }

    //forcing the simulation forward
    void callback_t_fwd_sim(const std_msgs::msg::Float64::SharedPtr msg){
        double t_sim2forward = msg->data;
        RCLCPP_INFO(this->get_logger(),"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        RCLCPP_INFO(this->get_logger(),"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        RCLCPP_INFO(this->get_logger(),"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        RCLCPP_INFO(this->get_logger(),"forwarding for %f mins!!!!",t_sim2forward/60.0);
        RCLCPP_INFO(this->get_logger(),"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        RCLCPP_INFO(this->get_logger(),"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        RCLCPP_INFO(this->get_logger(),"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        
        //int divide = 64;
        int divide = (int)(t_sim2forward / this->Ttorque_); 
        for(int ii = 0; ii < divide; ii++){
            //forward_orbit_dynamics(t_sim/(double)(divide));
            forward_orbit_dynamics(this->Ttorque_);
            if(ii % 60 == 1){
                std::this_thread::sleep_for(std::chrono::duration<double>(0.1)); 
            }
        }    
        RCLCPP_INFO(this->get_logger(),"forwarded");
    }


    Eigen::Vector3d calc_acc(const Eigen::Vector3d& pos_eci, const Eigen::Vector3d& thruster_force_eci){
        // Calculate acceleration

        // ---- gravity term ----
        // Distance [m]
        double r = pos_eci.norm();  
        Eigen::Vector3d acc_gravity = -this->mu / (r * r * r) * pos_eci;

        // ---- thruster term ----
        Eigen::Vector3d acc_thruster = thruster_force_eci / this->total_mass;
        
        return acc_gravity + acc_thruster;
    }


    void forward_orbit_dynamics(double Tfwd_sec){

        // integrate by Euler method
        Eigen::Vector3d pos_eci_old = this->pos_eci_cur;
        Eigen::Vector3d vel_eci_old = this->vel_eci_cur;
        Eigen::Vector3d acc_eci_old = this->acc_eci_cur;

        // ---- Thruster force ----
        // Thruster force at body frame
        Eigen::Vector3d thruster_force_bf;
        this->thrusterMat.thrusterToBody(this->bias_thruster_input, thruster_force_bf);

        // DCM of attitude
        auto att_dcm = quat2dcm(this->attitude_quat);

        // Transform BF->ECI
        Eigen::Vector3d thruster_force_eci = att_dcm * thruster_force_bf;

        // update position & velocity
        this->pos_eci_cur = pos_eci_old + vel_eci_old*Tfwd_sec;
        this->vel_eci_cur = vel_eci_old + acc_eci_old*Tfwd_sec;

        // update acceleration
        this->acc_eci_cur = this->calc_acc(pos_eci_old, thruster_force_eci);
    }

    
    void callback_attitude_quat(const geometry_msgs::msg::Quaternion::SharedPtr msg)
    {
        this->attitude_quat[0] = msg->x;
        this->attitude_quat[1] = msg->y;
        this->attitude_quat[2] = msg->z;
        this->attitude_quat[3] = msg->w;
    }

    void callback_bias_thruster_inp(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        size_t idx = 0;
        for (auto& value : msg->data) {
            if (std::isnan(value)) {
                RCLCPP_ERROR(this->get_logger(), "Received NaN value in bias thruster input");
                return;
            }
            this->bias_thruster_input(idx) = value;
            ++idx;
        }

    }

    void callback_orbit_dynamics(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        //compute attitude update
        forward_orbit_dynamics(this->Ttorque_); //Ttorque_ is simulation time period
    }

};

double get_time_double(rclcpp::Node::SharedPtr node) {
    rclcpp::Clock::SharedPtr clock = node->get_clock();
    rclcpp::Time tclock = clock->now();
    return ((double)(tclock.seconds()) + (double)(tclock.nanoseconds()) * 0.001 * 0.001 * 0.001);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<OrbitDynamicsNode>(options);
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}