
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include<iostream>
#include<fstream>
#include<string>
#include<map>
#include <iomanip>
#include<chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>


constexpr double PI = 3.141592653589793;

// The Earth radius [m]
constexpr double EARTH_RADIUS = 6378.14 * 1e3;
// [m^3 s^-2]
constexpr double G_ME = 3.986004418e14;


namespace EigenUtil {

    void print_vector3d(const Eigen::Vector3d& vec) {
        std::cout << "["
            << vec(0) << ", "
            << vec(1) << ", "
            << vec(2) << "]"
            << std::endl;
    }

    void print_matrix3d(const Eigen::Matrix3d& mat) {
        
        for (int i = 0; i < 3; ++i) {
            if (i == 0) {
                std::cout << "[";
            }
            else {
                std::cout << " ";
            }

            std::cout << "["
                << std::setw(10) << mat(i, 0) << " "
                << std::setw(10) << mat(i, 1) << " "
                << std::setw(10) << mat(i, 2);

            if (i == 2) {
                std::cout << "]]" << std::endl;
            }
            else {
                std::cout << "]," << std::endl;
            }
        }
    }

    Eigen::Vector3d from_std_vector(const std::vector<double>& vec){
        Eigen::Vector3d out_vec(3);
        for (size_t i=0; i<3; ++i){
            out_vec[i] = vec[i];
        }
        return out_vec;
    }
}


namespace Rotation {

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


    Eigen::Matrix3d euler2dcm(const Eigen::Vector3d& euler_vec) {
        auto sin_euler_vec = euler_vec.array().sin();
        auto cos_euler_vec = euler_vec.array().cos();

        double s0 = sin_euler_vec[0];
        double s1 = sin_euler_vec[1];
        double s2 = sin_euler_vec[2];
        double c0 = cos_euler_vec[0];
        double c1 = cos_euler_vec[1];
        double c2 = cos_euler_vec[2];

        Eigen::Matrix3d dcm_mat;

        dcm_mat.coeffRef(0, 0) = c1 * c2;
        dcm_mat.coeffRef(0, 1) = c1 * s2;
        dcm_mat.coeffRef(0, 2) = -s1;

        dcm_mat.coeffRef(1, 0) = -c0 * s2 + s0 * s1 * c2;
        dcm_mat.coeffRef(1, 1) = c0 * c2 + s0 * s1 * s2;
        dcm_mat.coeffRef(1, 2) = s0 * c1;

        dcm_mat.coeffRef(2, 0) = s0 * s2 + c0 * s1 * c2;
        dcm_mat.coeffRef(2, 1) = -s0 * c2 + c0 * s1 * s2;
        dcm_mat.coeffRef(2, 2) = c0 * c1;

        return dcm_mat;
    }

    Eigen::Vector4d dcm2quat(const Eigen::Matrix3d& dcm) {

        std::vector<double> temp_q_vec{
            std::sqrt(1.0 + dcm.coeff(0, 0) - dcm.coeff(1, 1) - dcm.coeff(2, 2)) / 2.0,
            std::sqrt(1.0 - dcm.coeff(0, 0) + dcm.coeff(1, 1) - dcm.coeff(2, 2)) / 2.0,
            std::sqrt(1.0 - dcm.coeff(0, 0) - dcm.coeff(1, 1) + dcm.coeff(2, 2)) / 2.0,
            std::sqrt(1.0 + dcm.coeff(0, 0) + dcm.coeff(1, 1) + dcm.coeff(2, 2)) / 2.0
        };

        Eigen::Vector4d quat_vec;

        std::vector<double>::iterator max_it = std::max_element(temp_q_vec.begin(), temp_q_vec.end());
        size_t max_idx = std::distance(temp_q_vec.begin(), max_it);


        if (max_idx == 0) {
            quat_vec.coeffRef(1) = dcm.coeff(0, 1) + dcm.coeff(1, 0);
            quat_vec.coeffRef(2) = dcm.coeff(0, 2) + dcm.coeff(2, 0);
            quat_vec.coeffRef(3) = dcm.coeff(1, 2) - dcm.coeff(2, 1);
        }
        else if (max_idx == 1) {
            quat_vec.coeffRef(0) = dcm.coeff(0, 1) + dcm.coeff(1, 0);
            quat_vec.coeffRef(2) = dcm.coeff(2, 1) + dcm.coeff(1, 2);
            quat_vec.coeffRef(3) = dcm.coeff(2, 0) - dcm.coeff(0, 2);
        }
        else if (max_idx == 2) {
            quat_vec.coeffRef(0) = dcm.coeff(2, 0) + dcm.coeff(0, 2);
            quat_vec.coeffRef(1) = dcm.coeff(2, 1) + dcm.coeff(1, 2);
            quat_vec.coeffRef(3) = dcm.coeff(0, 1) - dcm.coeff(1, 0);
        }
        else {
            quat_vec.coeffRef(0) = dcm.coeff(1, 2) - dcm.coeff(2, 1);
            quat_vec.coeffRef(1) = dcm.coeff(2, 0) - dcm.coeff(0, 2);
            quat_vec.coeffRef(2) = dcm.coeff(0, 1) - dcm.coeff(1, 0);
        }

        quat_vec *= (0.25 / temp_q_vec[max_idx]);
        quat_vec[max_idx] = temp_q_vec[max_idx];

        return quat_vec;
    }
};


inline constexpr double deg2rad(double deg) { return deg / 180.0 * PI; }
inline constexpr double rad2deg(double rad) { return rad / PI * 180.0; }


class FrameTransformer
{
private:
    Eigen::Matrix3d local_frame_rot_mat;
    Eigen::Matrix3d inv_local_frame_rot_mat;
    Eigen::Vector3d local_frame_ori_vec;

public:

    FrameTransformer() {
        this->local_frame_rot_mat = Eigen::Matrix3d::Identity();
        this->inv_local_frame_rot_mat = Eigen::Matrix3d::Identity();
        this->local_frame_ori_vec = Eigen::Vector3d::Zero();
    }

    FrameTransformer(const Eigen::Matrix3d& local_frame_rot_mat, const Eigen::Vector3d& local_frame_ori_vec) {
        this->local_frame_rot_mat = local_frame_rot_mat;
        this->local_frame_ori_vec = local_frame_ori_vec;
        this->inv_local_frame_rot_mat = local_frame_rot_mat.inverse();
    }

    Eigen::Vector3d get_local_pos(const Eigen::Vector3d& global_pos_vec) {
        return this->inv_local_frame_rot_mat * (global_pos_vec - this->local_frame_ori_vec);
    }

    Eigen::Vector3d get_global_pos(const Eigen::Vector3d& local_pos_vec) {
        return this->local_frame_rot_mat * local_pos_vec + this->local_frame_ori_vec;
    }

    void update_rot_mat(const Eigen::Matrix3d& local_frame_rot_mat) {
        this->local_frame_rot_mat = local_frame_rot_mat;
        this->inv_local_frame_rot_mat = local_frame_rot_mat.inverse();
    }

    void update_ori_vec(const Eigen::Vector3d& local_frame_ori_vec) {
        this->local_frame_ori_vec = local_frame_ori_vec;
    }

    const Eigen::Matrix3d& get_local_frame_rot_mat() {
        return this->local_frame_rot_mat;
    }

    const Eigen::Matrix3d& get_inv_local_frame_rot_mat() {
        return this->inv_local_frame_rot_mat;
    }

    const Eigen::Vector3d& get_local_frame_ori_vec() {
        return this->local_frame_ori_vec;
    }
};


Eigen::Vector4d quaternion_diff_equ(const Eigen::Vector4d& q_vec, const Eigen::Vector3d& w_vec) {
    // Differential equation of quqternion
    auto r = w_vec[0];
    auto p = w_vec[1];
    auto y = w_vec[2];

    Eigen::Matrix4d sqew_mat;
    sqew_mat << 0, +y, -p, +r,
        -y, 0, +r, +p,
        +p, -r, 0, +y,
        -r, -p, -y, 0;

    auto dq_vec = 0.5 * sqew_mat * q_vec;
    return dq_vec;
}


Eigen::Vector4d update_quaternion(Eigen::Vector4d& q_vec, Eigen::Vector3d& w_vec, double dt) {
    // Runge–Kutta method
    auto dt_h = dt / 2;

    auto k1_vec = quaternion_diff_equ(q_vec, w_vec);
    auto k2_vec = quaternion_diff_equ(q_vec + k1_vec * dt_h, w_vec);
    auto k3_vec = quaternion_diff_equ(q_vec + k2_vec * dt_h, w_vec);
    auto k4_vec = quaternion_diff_equ(q_vec + k3_vec * dt, w_vec);
    auto next_q_vec = q_vec + (k1_vec + k2_vec + k3_vec + k4_vec) / 6 * dt;

    return next_q_vec;
}


class SpaceStationPhysics : public rclcpp::Node
{

private:
    // -------- variables --------

    double t;

    // ---- Parameters of the Earth ----
    // Initial phase [rad]
    double earth_init_phase;
    // Angular Velocity of revolution [rad/s]
    double earth_revolution_w;

    double sun_earth_distance;

    // ---- Parameters of the Space Station ----  
    // initial phase [rad]
    double ss_init_phase;
    // Elevation + earth radius
    double ss_motion_radius;
    // Normal vector of SAP @BF
    Eigen::Vector3d ss_sap_normal_vec;
    // Anlugar velocity of revolution [rad/s] 
    double ss_revolution_w;

    FrameTransformer ss_plane_inertia_ft;

    // ---- Parameters of the Earth ----  

    Eigen::Vector3d earth_pos_vec;
    Eigen::Vector3d ss_pos_vec;

    // Global:SCI, Local:ECI
    FrameTransformer sci_eci_ft;
    // Global:ECI, Local:SSBF
    FrameTransformer eci_ssbf_ft;

    // Quaternion of Space Station
    Eigen::Vector4d ss_quaternion_vec;
    // Angular velocity of SS
    Eigen::Vector3d ss_w_vec;

    // ---- Parameters of SAP ----
    // Maximum power generation [W]
    double max_generated_power;
    // Full amount of battery [Wh]
    double full_battery_amount;
    // Maximum power generation [W]
    double cur_generated_power;
    // Current amount of battery [Wh]
    double cur_battery_amount;
    // Consumed power by other subsystem [W]
    double ss_power_consumption;

    // 
    Eigen::Vector3d ss_sap_basic_normal_vec = Eigen::Vector3d(0.0, 0.0, -1.0);
    // Solar array rotary joint angle [rad]
    double sarj_angle;

    // Time of start simulation
    std::chrono::system_clock::time_point simu_start_time;

    double simu_speed_rate ;

    // ---- Publishers ----
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_generated_power_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_soc_;

    rclcpp::TimerBase::SharedPtr timer_;

    Eigen::Vector3d calc_earth_pos_vec(double t) {
        // -------- Calculate earth position @SCI at t --------
        double cur_phase = this->earth_init_phase + this->earth_revolution_w * t;
        Eigen::Vector3d earth_pos_vec(
            std::cos(cur_phase) * this->sun_earth_distance,
            std::sin(cur_phase) * this->sun_earth_distance,
            0.0
        );
        return earth_pos_vec;
    }

    Eigen::Vector3d calc_ss_pos_vec(double t) {
        // -------- Calculate SS position @ECI at t --------
        // Current SS phase at ECI
        double cur_phase = this->earth_init_phase + this->ss_revolution_w * t;

        // Position at orbital plane
        Eigen::Vector3d ss_pos_plane_vec(
            std::cos(cur_phase) * this->ss_motion_radius,
            std::sin(cur_phase) * this->ss_motion_radius,
            0.0
        );

        // 
        return this->ss_plane_inertia_ft.get_global_pos(ss_pos_plane_vec);
    }

public:

    void initialize(
        double ss_altitude, double ss_raan, double ss_inclination,
        const Eigen::Vector3d& ss_init_euler_vec,
        const Eigen::Vector3d& ss_init_w_vec,
        double simu_timestep, double simu_speed_rate
    )
    {
        this->sun_earth_distance = 149600000 * 1e3;

        // ---- Parameters of the Earth ----
        this->earth_init_phase = 0.0;
        this->earth_revolution_w = 2.0 * PI / (365.25 * 24 * 60 * 60);

        // ---- Parameters of the Space Station ----
        this->ss_init_phase = 0.0;
        this->ss_motion_radius = EARTH_RADIUS + ss_altitude;
        // Velocity of the SS [m/s]
        double ss_v = std::sqrt(G_ME / this->ss_motion_radius);
        double ss_revolution_period = 2.0 * PI * this->ss_motion_radius / ss_v;
        this->ss_revolution_w = 2.0 * PI / ss_revolution_period;

        Eigen::Matrix3d ss_plane_inertia_rot_mat = Rotation::euler2dcm(
            Eigen::Vector3d(ss_raan, 0.0, ss_inclination)
        );
        this->ss_plane_inertia_ft = FrameTransformer(
            ss_plane_inertia_rot_mat.transpose(), Eigen::Vector3d::Zero()
        );

        // -------- Initialize time-varying parameters --------
        this->t = 0.0;

        this->earth_pos_vec = this->calc_earth_pos_vec(this->t);
        this->ss_pos_vec = this->calc_ss_pos_vec(this->t);

        this->sci_eci_ft = FrameTransformer(
            Eigen::Matrix3d::Identity(), this->earth_pos_vec
        );
        this->eci_ssbf_ft = FrameTransformer(
            Eigen::Matrix3d::Identity(), this->ss_pos_vec
        );

        Eigen::Matrix3d ss_attitude_rot_mat = Rotation::euler2dcm(ss_init_euler_vec);//.transpose();
        this->ss_quaternion_vec = Rotation::dcm2quat(ss_attitude_rot_mat);
        this->ss_w_vec = ss_init_w_vec;

        // ---- Power generation ----
        this->max_generated_power = 1000.0 / 6;
        this->full_battery_amount = 1000.0 * 2 * 60 * 60;
        this->cur_battery_amount = this->full_battery_amount;
        this->ss_power_consumption = 100.0;

        this->sarj_angle = 0.0;

        this->ss_sap_normal_vec = this->calc_ss_sap_normal_vec();

        // ---- Other -----
        this->simu_start_time = std::chrono::system_clock::now();

        this->simu_speed_rate = simu_speed_rate;

        // Publish period [milli second]
        int32_t ros2_publish_period_ms = int32_t(simu_timestep / this->simu_speed_rate * 1000);
        std::cout << ros2_publish_period_ms << std::endl;
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(ros2_publish_period_ms),
            std::bind(&SpaceStationPhysics::publish_value, this)
        );

        // ---- Publishers ----
        this->publisher_generated_power_ = this->create_publisher<std_msgs::msg::Float64>("generated_power", 10);
        this->publisher_soc_ = this->create_publisher<std_msgs::msg::Float64>("soc", 10);
    }
    
    SpaceStationPhysics() : Node("space_station_power_demo")
    {
        // -------- Declare parameters and set default value --------
        this->declare_parameter<double>("ss_altitude", 400 * 1e3);
        this->declare_parameter<double>("ss_raan", deg2rad(0.0));
        this->declare_parameter<double>("ss_inclination", deg2rad(20.0));

        this->declare_parameter<std::vector<double>>("ss_init_euler_angle", {0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("ss_init_w_vec", {0.0, 0.02, 0.0});

        this->declare_parameter<double>("simu_timestep", 2*60);
        this->declare_parameter<double>("simu_speed_rate", 1000);

        // -------- Get parameters --------
        double ss_altitude = this->get_parameter("ss_altitude").as_double();
        double ss_raan = this->get_parameter("ss_raan").as_double();
        double ss_inclination = this->get_parameter("ss_inclination").as_double();

        std::vector<double> temp_ss_init_euler_vec = this->get_parameter("ss_init_euler_angle").as_double_array();
        std::vector<double> temp_ss_init_w_vec = this->get_parameter("ss_init_w_vec").as_double_array();
        
        double simu_timestep = this->get_parameter("simu_timestep").as_double();
        double simu_speed_rate = this->get_parameter("simu_speed_rate").as_double();

        // -------- Convert --------
        Eigen::Vector3d ss_init_euler_vec = EigenUtil::from_std_vector(temp_ss_init_euler_vec);
        Eigen::Vector3d ss_init_w_vec = EigenUtil::from_std_vector(temp_ss_init_w_vec);
        
        this->initialize(
            ss_altitude, ss_raan, ss_inclination,
            ss_init_euler_vec, ss_init_w_vec,
            simu_timestep, simu_speed_rate
        );
    }

    void update(double new_t) {

        // Simulation time
        double dt = new_t - this->t;
        this->t = new_t;

        // -------- Store old values --------
        Eigen::Vector3d old_sap_normal_vec = this->ss_sap_normal_vec;
        Eigen::Vector3d old_sun_direction_vec = this->get_sun_pos_at_ss_vec();

        // -------- Dynamics --------

        // ---- Position ----
        this->earth_pos_vec = this->calc_earth_pos_vec(this->t);
        this->ss_pos_vec = this->calc_ss_pos_vec(this->t);

        // ---- Attitude ----
        this->ss_quaternion_vec = update_quaternion(this->ss_quaternion_vec, this->ss_w_vec, dt);
        Eigen::Matrix3d ss_rot_mat = Rotation::quat2dcm(this->ss_quaternion_vec);

        // Frame transformer of SCI - ECI
        this->sci_eci_ft.update_ori_vec(this->earth_pos_vec);
        // Frame transformer of ECI - BF
        this->eci_ssbf_ft.update_rot_mat(ss_rot_mat.transpose());
        this->eci_ssbf_ft.update_ori_vec(this->ss_pos_vec);


        // -------- Power --------

        // ---- Update Solar array direction by SARJ angle ----
        this->ss_sap_normal_vec = this->calc_ss_sap_normal_vec();

        double cos_theta = old_sap_normal_vec.dot(old_sun_direction_vec.normalized());

        this->cur_generated_power = (cos_theta > 0) ? this->max_generated_power * cos_theta : 0.0;

        this->cur_battery_amount -= this->ss_power_consumption * dt;
        this->cur_battery_amount += this->cur_generated_power * dt;
        if (this->cur_battery_amount < 0) {
            this->cur_battery_amount = 0.0;
        }
        else if (this->full_battery_amount < this->cur_battery_amount) {
            this->cur_battery_amount = this->full_battery_amount;
        }

        return;
    }

    void publish_value() {
        // Get current time
        std::chrono::system_clock::time_point simu_cur_time = std::chrono::system_clock::now();
        double elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(simu_cur_time - this->simu_start_time).count();
        double simu_time = elapsed_time * this->simu_speed_rate;

        this->update(simu_time);

        // -------- Publish --------

        // ---- Convert ---- 
        auto message_cur_generated_power = std_msgs::msg::Float64();
        auto message_soc = std_msgs::msg::Float64();

        message_cur_generated_power.data = this->cur_generated_power;
        message_soc.data = this->cur_battery_amount;

        RCLCPP_INFO(this->get_logger(), "Publishing: generated_power=%f[W], battery_amount=%f[Wh]", this->cur_generated_power, this->cur_battery_amount);

        // ---- Publish ----
        this->publisher_generated_power_->publish(message_cur_generated_power);
        this->publisher_soc_->publish(message_soc);
    }

    Eigen::Vector3d calc_ss_sap_normal_vec() {
        //

        Eigen::Matrix3d sarj_rot_mat = Eigen::Matrix3d::Identity();
        sarj_rot_mat.coeffRef(0, 0) = std::cos(this->sarj_angle);
        sarj_rot_mat.coeffRef(0, 2) = std::sin(this->sarj_angle);
        sarj_rot_mat.coeffRef(2, 0) = -std::sin(this->sarj_angle);
        sarj_rot_mat.coeffRef(2, 2) = std::cos(this->sarj_angle);

        Eigen::Vector3d ss_sap_normal_vec = sarj_rot_mat * this->ss_sap_basic_normal_vec;
        return ss_sap_normal_vec;
    }

    // -------- For Hardware Model --------

    inline double get_time() const {
        return this->t;
    }

    Eigen::Vector3d get_ss_sap_normal_vec() {
        return this->ss_sap_normal_vec;
    }

    void set_sarj_angle(double sarj_angle) {
        this->sarj_angle = sarj_angle;
    }

    Eigen::Vector3d get_sun_pos_at_ss_vec() {
        // -------- Calculate sun position vector at BF --------
        Eigen::Vector3d sun_pos_eci_vec = this->sci_eci_ft.get_local_pos(Eigen::Vector3d::Zero());
        // Position vector of the Sun seen from the origin of BF-frame.
        // Each axis direction is same as ECI.
        Eigen::Vector3d sun_pos_ss_center_vec = sun_pos_eci_vec - this->eci_ssbf_ft.get_local_frame_ori_vec();
        // Normalization (if don't normalize, value becomes bad when rotation matrix is multiplied.)
        Eigen::Vector3d sun_pos_ss_center_normal_vec = sun_pos_ss_center_vec / sun_pos_ss_center_vec.norm();
        Eigen::Vector3d sun_pos_ssbf_vec = this->eci_ssbf_ft.get_inv_local_frame_rot_mat() * sun_pos_ss_center_normal_vec;
        return sun_pos_ssbf_vec;
    }

    double get_battery_soc() const {
        return this->cur_battery_amount / this->full_battery_amount;
    }
};


int main(int argc, char* argv[]){
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpaceStationPhysics>());
    rclcpp::shutdown();
    
    return 1;
}

