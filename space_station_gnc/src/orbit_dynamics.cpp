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
        // // dynamics parameters
        // this->declare_parameter<double>("dynamics.J.xx", 280e6);
        // this->declare_parameter<double>("dynamics.J.yy", 140e6);
        // this->declare_parameter<double>("dynamics.J.zz", 420e6);
        // this->declare_parameter<double>("dynamics.mu", 3.986e14);
        // this->declare_parameter<double>("dynamics.r_orbit", 7e6);
        
        // // timing parameters
        // this->declare_parameter<double>("timing.torque_dt", 0.1);
        // this->declare_parameter<double>("timing.pub_dt", 0.1);
        // this->declare_parameter<int>   ("timing.publish_every", 10);
        
        // // initial state parameters
        // this->declare_parameter<std::vector<double>>("initial.attitude", {0,0,0,1});
        // this->declare_parameter<std::vector<double>>("initial.angvel", {0,0,0});
        // this->declare_parameter<std::vector<double>>("initial.angacc", {0,0,0});



        this->Ttorque_ = this->get_parameter("timing.torque_dt").as_double();
        this->Tpubatt_ = this->get_parameter("timing.pub_dt").as_double();
        this->N2disp   = this->get_parameter("timing.publish_every").as_int();

        auto pos0 = this->get_parameter("initial.position").as_double_array();
        this->pos_eci_cur = Eigen::Vector3(pos0[0], pos0[1], pos0[2]);

        auto vel0 = this->get_parameter("initial.velocity").as_double_array();
        this->vel_eci_cur = Eigen::Vector3(vel0[0], vel0[1], vel0[2]);

        auto acc0 = this->get_parameter("initial.acceleration").as_double_array();
        this->acc_eci_cur = Eigen::Vector3(acc0[0], acc0[1], acc0[2]);

        this->pub_pos_eci = this->create_publisher<geometry_msgs::msg::Vector3>("gnc/pos_eci", 10);
        // this->pub_vel_eci = this->create_publisher<geometry_msgs::msg::Vector3>("gnc/vel_eci", 10);
        // this->pub_acc_eci = this->create_publisher<geometry_msgs::msg::Vector3>("gnc/acc_eci", 10);
        
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        this->i2disp = 0; //publish rate: Npublish * Ttorque
        this->should_pub_att = false;

        printq(attcur);
        
        // ---- Dynamics Parameter ----
        
        //TODO: handled by parameter surver?
        this->total_mass = this->get_parameter("dynamics.total_mass").as_double();
        mu = this->get_parameter("dynamics.mu").as_double();

        // ---- Subscription ----

        this->sub_t_fwd_sim = this->create_subscription<std_msgs::msg::Float64>(
            "gnc/t_fwd_sim", 1, 
            std::bind(&AttitudeDynamicsNode::callback_t_fwd_sim, this, std::placeholders::_1));

        this->sub_torque_control = this->create_subscription<geometry_msgs::msg::Vector3>(
            "gnc/thr_torque_cmd", 1, 
            std::bind(&AttitudeDynamicsNode::callback_attitude_dynamics, this, std::placeholders::_1));
        
        this->sub_bias_thruster_control = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "gnc/bias_thruster_cmd", 1, 
            std::bind(&AttitudeDynamicsNode::callback_bias_thruster_inp, this, std::placeholders::_1));
        
        this->sub_attitude_quat = this->create_subscription<std_msgs::msg::Quaternion>(
            "gnc/attitude_LVLH", 1, 
            std::bind(&AttitudeDynamicsNode::callback_attitude_quat, this, std::placeholders::_1));
        
        // ---- Publish timer ----

        timer_pos_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(Tpubatt_*1000.0)), 
            std::bind(&AttitudeDynamicsNode::callback_timer_pub_pos, this));

        // ---- Initialize variables ----
        this->bias_thruster_input = Eigen::VectorXd(this->n_thruster);
    }

    private:

    const size_t n_thruster = 12;
    
    //ros2 stuff
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_t_fwd_sim;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_bias_thruster_control;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_torque_control;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr sub_attitude_quat;

    bool received_ = false;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_pos_eci;
    // rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_vel_eci;
    // rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_acc_eci;

    rclcpp::TimerBase::SharedPtr timer_pos_;

    //define parameters for dynamics
    // as well as CoG offset issue is a big TODO
    double mu;               ///< Gravitational parameter (m^3/s^2)

    // Position, velocity and acceleration of space station at Earth Centered Inertial Frame (ECI) [m], [m/s], [m/s^2]
    Eigen::Vector3d pos_eci_cur;
    Eigen::Vector3d vel_eci_cur;
    Eigen::Vector3d acc_eci_cur;

    // thruster force [N]
    Eigen::VectorXd bias_thruster_input;

    // Attitude Quaternion
    Eigen::Vector4d attitude_quat;

    //define parameters for simulation
    double Tstep_rk; // in seconds
    double Ttorque_; //same as T_callback
    //int Nstep; // number of steps for simulation, Ttorque_/Tstep_

    int N2disp; // publish rate: Npublish * Ttorque
    int i2disp; // index for i2disp
    double Tpubatt_; 

    ThrusterMatrix thrusterMat;

    // -------- Callback functions --------

    void callback_timer_pub_pos() {

        geometry_msgs::msg::Vector3 pos_eci_msg;
        pos_eci_msg.x = this->pos_eci_cur.x;
        vel_eci_msg.y = this->vel_eci_cur.y;
        acc_eci_msg.z = this->acc_eci_cur.z;
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
            //forward_attitude_dynamics(t_sim/(double)(divide));
            forward_attitude_dynamics(this->Ttorque_);
            if(ii % 60 == 1){
                std::this_thread::sleep_for(std::chrono::duration<double>(0.1)); 
            }
        }    
        RCLCPP_INFO(this->get_logger(),"forwarded");
    }


    Eigen::Vector3d calc_acc(const Eigen::Vector3d& pos_eci, const Eigen::Vector3d& thruster_force_eci){
        // Calculate acceleration

        // by gravity
        double r = pos_eci.norm();  // 距離 [m]
        Eigen::Vector3d acc_gravity = -this->mu / (r * r * r) * pos_eci;

        // by thruster
        Eigen::Vector3d acc_thruster = thruster_force_eci / this->total_mass;
        return acc_gravity + acc_thruster;
    }


    void forward_attitude_dynamics(double Tfwd_sec){

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

    
    void callback_attitude_quat(const geometry_msgs::msg::Quaterniond msg)
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

    void callback_attitude_dynamics(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        //compute attitude update
        forward_attitude_dynamics(this->Ttorque_); //Ttorque_ is simulation time period
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