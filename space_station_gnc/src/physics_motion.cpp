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
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <vector>
#include "L_p_func.cpp"
#include "space_station_gnc/action/unloading.hpp"
//std::string mode_demo;

class AttitudeDynamicsNode : public rclcpp::Node
{
public:
    using unloading =  space_station_gnc::action::Unloading;
    using GoalHandleUnloading = rclcpp_action::ClientGoalHandle<unloading>;
    AttitudeDynamicsNode(const rclcpp::NodeOptions & options) : Node("physics_motion", options)
    {
        // dynamics parameters
        this->declare_parameter<double>("dynamics.J.xx", 280e6);
        this->declare_parameter<double>("dynamics.J.yy", 140e6);
        this->declare_parameter<double>("dynamics.J.zz", 420e6);
        this->declare_parameter<double>("dynamics.mu", 3.986e14);
        this->declare_parameter<double>("dynamics.r_orbit", 7e6);
        
        // timing parameters
        this->declare_parameter<double>("timing.torque_dt", 0.1);
        this->declare_parameter<double>("timing.pub_dt", 0.1);
        this->declare_parameter<int>   ("timing.publish_every", 10);
        
        // initial state parameters
        this->declare_parameter<std::vector<double>>("initial.attitude", {0,0,0,1});
        this->declare_parameter<std::vector<double>>("initial.angvel", {0,0,0});
        this->declare_parameter<std::vector<double>>("initial.angacc", {0,0,0});

        Ttorque_ = this->get_parameter("timing.torque_dt").as_double();
        Tpubatt_ = this->get_parameter("timing.pub_dt").as_double();
        N2disp   = this->get_parameter("timing.publish_every").as_int();

        auto att0 = this->get_parameter("initial.attitude").as_double_array();
        attcur = Eigen::Quaterniond(att0[3], att0[0], att0[1], att0[2]);
        
        auto ang0 = this->get_parameter("initial.angvel").as_double_array();
        omebcur.setValue(ang0[0], ang0[1], ang0[2]);
        
        auto acc0 = this->get_parameter("initial.angacc").as_double_array();
        omedotbcur.setValue(acc0[0], acc0[1], acc0[2]);

        H_th << 0.3,0.3,0.3;

        pub_attitude_LVLH = this->create_publisher<geometry_msgs::msg::Quaternion>("gnc/attitude_LVLH", 10);
        pub_angvel_body = this->create_publisher<geometry_msgs::msg::Vector3>("gnc/angvel_body", 10);
        pub_cmg_del = this->create_publisher<std_msgs::msg::Float64MultiArray>("gnc/cmg_del", 1);
        pub_pose_all = this->create_publisher<geometry_msgs::msg::PoseStamped>("gnc/pose_all", 10);
        pub_cmg_h = this->create_publisher<geometry_msgs::msg::Vector3>("gnc/cmg_h", 1);

        pub_ang_acc = this->create_publisher<geometry_msgs::msg::Vector3>("gnc/ang_acc", 10);
        //pub_pose_marker = this->create_publisher<visualization_msgs::msg::Marker>("pose_marker", 10);
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);


        i2disp = 0; //publish rate: Npublish * Ttorque
        should_pub_att = false;

        //omebcur.setValue(0.0,+0.22 / 180.0 * M_PI,0.0);
        omebprv = omebcur;
        //omedotbcur.setValue(0.0,+0.0003 / 180.0 * M_PI,0.0);
        omedotbcur.setValue(0.0,+0.0000780,0.0);
        omedotbprv = omedotbcur;
        publish_attitude(attcur);
        printq(attcur);
        
        //attcur.setRPY(0.0,1.72/180.0*M_PI,0.0);
        //attcur.setRPY(0.0,11.97/180.0*M_PI,0.0);
        attprv = attcur;
        
        //TODO: handled by parameter surver?
        double J11 = this->get_parameter("dynamics.J.xx").as_double();
        double J22 = this->get_parameter("dynamics.J.yy").as_double();
        double J33 = this->get_parameter("dynamics.J.zz").as_double();
        J123 << J11, 0.0, 0.0,
                0.0, J22, 0.0,
                0.0, 0.0, J33;
        J123inv = J123.inverse();
        mu = this->get_parameter("dynamics.mu").as_double();
        rOrbit = this->get_parameter("dynamics.r_orbit").as_double();

        deltacur << 0.0, 0.0, 0.0, 0.0;

        sub_t_fwd_sim = this->create_subscription<std_msgs::msg::Float64>(
            "gnc/t_fwd_sim", 1, 
            std::bind(&AttitudeDynamicsNode::callback_t_fwd_sim, this, std::placeholders::_1));

        sub_attitude_overwrite = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "gnc/attitude_overwrite", 1, 
            std::bind(&AttitudeDynamicsNode::callback_attitude_overwrite, this, std::placeholders::_1));

        sub_angvel_overwrite = this->create_subscription<geometry_msgs::msg::Vector3>(
            "gnc/angvel_overwrite", 1, 
            std::bind(&AttitudeDynamicsNode::callback_angvel_overwrite, this, std::placeholders::_1));

        sub_torque_control = this->create_subscription<geometry_msgs::msg::Vector3>(
            "gnc/thr_torque_cmd", 1, 
            std::bind(&AttitudeDynamicsNode::callback_attitude_dynamics, this, std::placeholders::_1));
        
        sub_cmg_torque_control = this->create_subscription<geometry_msgs::msg::Vector3>(
            "gnc/cmg_torque_cmd", 1, 
            std::bind(&AttitudeDynamicsNode::callback_cmg_inp, this, std::placeholders::_1));
        
        sub_bias_torque_control = this->create_subscription<geometry_msgs::msg::Vector3>(
            "gnc/bias_torque_cmd", 1, 
            std::bind(&AttitudeDynamicsNode::callback_bias_inp, this, std::placeholders::_1));
            
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(Tpubatt_*1000.0)), 
            std::bind(&AttitudeDynamicsNode::callback_timer_pub_att, this));

        timer_acc_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(Tpubatt_*1000.0)), 
            std::bind(&AttitudeDynamicsNode::callback_timer_pub_acc, this));
        
        timer_unload_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(Tpubatt_*1000.0)), 
            std::bind(&AttitudeDynamicsNode::callback_timer_pub_unload, this));

        unloading_client_ = rclcpp_action::create_client<unloading>(
            this, "gnc/unloading");

        if (!unloading_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Unloading action server not available");
            rclcpp::shutdown();
        }

        goal_msg.unload = false;
        send_goal_options.goal_response_callback = std::bind(&AttitudeDynamicsNode::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&AttitudeDynamicsNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&AttitudeDynamicsNode::result_callback, this, std::placeholders::_1);
    }

private:
    
    //ros2 stuff
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_torque_control;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_cmg_torque_control;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_bias_torque_control;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_t_fwd_sim;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_angvel_overwrite;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_attitude_overwrite;

    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_attitude_LVLH;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_angvel_body;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmg_del;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_all;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_cmg_h;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_ang_acc;
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_pose_marker;
    geometry_msgs::msg::Quaternion attitude_LVLH;
    geometry_msgs::msg::Vector3 angvel_body;
    geometry_msgs::msg::Vector3 ang_acc;
    std_msgs::msg::Float64MultiArray cmg_del;
    geometry_msgs::msg::Vector3 cmg_h;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_acc_;
    rclcpp::TimerBase::SharedPtr timer_unload_;
    rclcpp_action::Client<unloading>::SharedPtr unloading_client_;

    //define parameters for dynamics
    //TODO: Jall, dynamic change of J, 
    // as well as CoG offset issue is a big TODO
    Eigen::Matrix3d J123;
    Eigen::Matrix3d J123inv;
    double mu;               ///< Gravitational parameter (m^3/s^2)
    double rOrbit;           ///< Nominal orbit radius (m)



    Eigen::Quaterniond attcur; // pose
    Eigen::Quaterniond attprv; // pose
    tf2::Vector3 omebcur; //rad/sec, angular velocity of body frame
    tf2::Vector3 omebprv; //rad/sec, angular velocity of body frame
    tf2::Vector3 omedotbcur; //rad/sec^2 angular acc of body frame
    tf2::Vector3 omedotbprv; //rad/sec^2 angular acc of body frame
    Eigen::Vector4d deltacur; //rad, each CMG angle around torque axis

    
    tf2::Vector3 tau_ctlcmgcur; 
    tf2::Vector3 tau_ctlthrcur; 
    tf2::Vector3 tau_extgracur; 
    tf2::Vector3 tau_allcur; 
    tf2::Vector3 tau_ctlbiascur;



    //define parameters for simulation
    double Tstep_rk; // in seconds
    double Ttorque_; //same as T_callback
    //int Nstep; // number of steps for simulation, Ttorque_/Tstep_

    int N2disp; // publish rate: Npublish * Ttorque
    int i2disp; // index for i2disp
    double Tpubatt_; 
    Eigen::Vector3d H_th;
    rclcpp_action::ClientGoalHandle<unloading>::SharedPtr goal_handle;
    unloading::Goal goal_msg;
    rclcpp_action::Client<unloading>::SendGoalOptions send_goal_options;
    // rclcpp_action::Client<unloading>::CancelGoalOptions cancel_goal_options;

    bool should_pub_att; // true then publish attitude for display purpose

    std::atomic<bool> unload = false; // Flag to indicate if unloading is required
    std::atomic<bool> prevUnload = false;

    void goal_response_callback(
        const GoalHandleUnloading::SharedPtr &future)
    {
        goal_handle = future;
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the unloading server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by the unloading server");
        }
    }

    void feedback_callback(
        GoalHandleUnloading::SharedPtr,
        const std::shared_ptr<const unloading::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Feedback received: %f", feedback->rem);
    }

    void result_callback(const GoalHandleUnloading::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Successfully Unloaded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Unloading action aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Action canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown action result code");
                break;
        }
    }


    //utility function
    void printq(const Eigen::Quaterniond& q){
        Eigen::Matrix<double,3,3> m = q.toRotationMatrix(); //r11,r21,r31 show where original x axis points
        double r11 = m(0,0), r12 = m(0,1), r13 = m(0,2);
        double r21 = m(1,0), r22 = m(1,1), r23 = m(1,2);
        double r31 = m(2,0), r32 = m(2,1), r33 = m(2,2);
        RCLCPP_INFO(this->get_logger(),"  [%+.4f, %+.4f, %+.4f]",r11,r12,r13);
        RCLCPP_INFO(this->get_logger(),"  [%+.4f, %+.4f, %+.4f]",r21,r22,r23);
        RCLCPP_INFO(this->get_logger(),"  [%+.4f, %+.4f, %+.4f]",r31,r32,r33);
    }

    //utility function
    void publish_attitude(Eigen::Quaterniond att){
        attitude_LVLH.x = att.x();
        attitude_LVLH.y = att.y();
        attitude_LVLH.z = att.z();
        attitude_LVLH.w = att.w();
        pub_attitude_LVLH->publish(attitude_LVLH);   //rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_attitude_LVLH;
            
        auto message = geometry_msgs::msg::PoseStamped();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "world";
        message.pose.orientation.x = att.x();
        message.pose.orientation.y = att.y();
        message.pose.orientation.z = att.z();
        message.pose.orientation.w = att.w();
        pub_pose_all->publish(message);    //rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_all;

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "world";
        //transform_stamped.child_frame_id = "pose_marker";
        transform_stamped.child_frame_id = "Root";

        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = att.x();
        transform_stamped.transform.rotation.y = att.y();
        transform_stamped.transform.rotation.z = att.z();
        transform_stamped.transform.rotation.w = att.w();

        broadcaster_->sendTransform(transform_stamped);        
    }

    // utility function to calculate total angular momentum
    Eigen::Vector3d compute_h(const Eigen::Vector4d& delta) {    
        Eigen::Vector3d h;        
        const casadi_real* deltaI[4] = {&delta(0), &delta(1), &delta(2), &delta(3)};

        std::vector<casadi_real> h_out(3, 0.0);  
        casadi_real* resH[1] = {h_out.data()};

        casadi_int iwH[70] = {0}; 
        casadi_real wH[70] = {0};
        int mem = 0;

        hFunc(deltaI, resH, iwH, wH, mem);

        casadi_real* hArr = &h_out[0];
        h = Eigen::Map<Eigen::Vector3d>(hArr);
        return h;
    }
    
    //utility function to calculate pseudo-inverse matrix
    Eigen::Matrix<double,4,3> pseudoinverse(Eigen::Vector4d delta) {
        Eigen::Matrix<double,4,3> pseudoinv;        
        const casadi_real* deltaI[4] = {&delta(0), &delta(1), &delta(2), &delta(3)};

        std::vector<casadi_real> Inv(12, 0.0);  
        casadi_real* resInv[1] = {Inv.data()};

        casadi_int iwInv[70] = {0}; 
        casadi_real wInv[70] = {0};
        int mem = 0;

        pseudoInvFunc(deltaI, resInv, iwInv, wInv, mem);

        casadi_real* invArr = &Inv[0];
        pseudoinv = Eigen::Map<Eigen::Matrix<double,4,3>>(invArr);

        return pseudoinv;
    }

    void callback_timer_pub_att() {
        if(!should_pub_att)
            return;
        publish_attitude(attcur);

        //
        angvel_body.x = omebcur.x();
        angvel_body.y = omebcur.y();
        angvel_body.z = omebcur.z();
        pub_angvel_body->publish(angvel_body); 
        
        cmg_del.data = {deltacur(0), deltacur(1), deltacur(2), deltacur(3)};
        pub_cmg_del->publish(cmg_del);

    }

    void callback_timer_pub_acc() {
        // Calculate angular acceleration in body frame
        Eigen::Vector3d tau_allcure(tau_allcur.x(), tau_allcur.y(), tau_allcur.z());
        // for current 
        Eigen::Vector3d tau_inp(tau_ctlcmgcur.x(), tau_ctlcmgcur.y(), tau_ctlcmgcur.z());
        Eigen::Vector3d omega_inp(omebcur.x(), omebcur.y(), omebcur.z());
            
        Eigen::Vector3d rhs1 = J123 * omega_inp;
        Eigen::Vector3d tau_eff = tau_allcure - omega_inp.cross(rhs1);
        Eigen::Vector3d omega_dot_acc = J123inv * tau_eff;

        ang_acc.x = omega_dot_acc.x();
        ang_acc.y = omega_dot_acc.y();
        ang_acc.z = omega_dot_acc.z();

        pub_ang_acc->publish(ang_acc);
    }

    void callback_timer_pub_unload() {
        Eigen::Vector3d h = compute_h(deltacur);
        cmg_h.x = h.x();
        cmg_h.y = h.y();
        cmg_h.z = h.z();
        pub_cmg_h->publish(cmg_h);

        unload = checkCMGThresh(h);
        if (unload && !prevUnload) {
            RCLCPP_INFO(this->get_logger(), "CMG threshold exceeded, unloading initiated.");
            goal_msg.unload = unload;
            unloading_client_->async_send_goal(goal_msg,send_goal_options);
        }
        else if (!unload && prevUnload) {
            RCLCPP_INFO(this->get_logger(), "Unloading stopped.");
            unloading_client_->async_cancel_goal(goal_handle);
        }
        prevUnload = unload.load();
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
        int divide = (int)(t_sim2forward / Ttorque_); 
        for(int ii = 0; ii < divide; ii++){
            //forward_attitude_dynamics(t_sim/(double)(divide));
            forward_attitude_dynamics(Ttorque_);
            if(ii % 60 == 1){
                publish_attitude(attcur);
                std::this_thread::sleep_for(std::chrono::duration<double>(0.1)); 
            }
        }    
        publish_attitude(attcur);
        RCLCPP_INFO(this->get_logger(),"forwarded");
    }


    //utility function
    tf2::Quaternion stepupdate_attitude(const tf2::Quaternion& qi, const tf2::Vector3& omeb, double dt) 
    {
        // Step 1: Calculate rotation angle
        double thetab = omeb.length() * dt;
        if(thetab == 0.0)
            return qi;

        // Step 2: Calculate rotation axis (unit vector)
        tf2::Vector3 u = omeb.normalized();

        // Step 3: Calculate rotation quaternion in the body frame
        double half_thetab = thetab / 2.0;
        double sin_half_thetab = std::sin(half_thetab);
        tf2::Quaternion q_omegab(
            u.x() * sin_half_thetab,
            u.y() * sin_half_thetab,
            u.z() * sin_half_thetab,
            std::cos(half_thetab)
        );
        q_omegab = q_omegab.normalize();

        // Step 4: Transform q_omegab to world frame
        tf2::Quaternion q_omega_world = qi * q_omegab * qi.inverse();

        // Step 5: Calculate new orientation quaternion
        tf2::Quaternion qf = q_omega_world * qi;
        qf = qf.normalize();
        return qf;
    }

    bool checkCMGThresh(const Eigen::Vector3d& hState)
    {
        bool exceed = (std::abs(hState.x()) > H_th.x()) 
                    || (std::abs(hState.y()) > H_th.y())
                    || (std::abs(hState.z()) > H_th.z());

        return exceed;
    }

    void forward_attitude_dynamics(double Tfwd_sec){
        double thetab = omebcur.length();
        if (0.5 < thetab)
            Tstep_rk = 0.001;
        else if(0.25 < thetab)
            Tstep_rk = 0.05;
        else if(0.1 < thetab)
            Tstep_rk = 0.02;
        else 
            Tstep_rk = 0.01;

        int Nstep = (int)(Tfwd_sec/Tstep_rk);
        
        Eigen::Vector3d tau_allcure(tau_allcur.x(), tau_allcur.y(), tau_allcur.z());

        for(int istep = 0; istep < Nstep; istep++){
            // for current 
            Eigen::Vector3d tau_inp(tau_ctlcmgcur.x(), tau_ctlcmgcur.y(), tau_ctlcmgcur.z());
            Eigen::Vector3d omega_k1(omebcur.x(), omebcur.y(), omebcur.z());
            Eigen::Quaterniond att_k1 = attcur;
            Eigen::Vector4d delta_k1 = deltacur;
            
            // for RK4 k1 
            Eigen::Vector3d rhs1 = J123 * omega_k1;
            Eigen::Vector3d tau_eff = tau_allcure - omega_k1.cross(rhs1);
            Eigen::Vector3d omega_dot_k1 = J123inv * tau_eff;
            Eigen::Matrix<double,4,3> pseudoInv1 = pseudoinverse(delta_k1);
            Eigen::Vector3d h1 = compute_h(delta_k1);
            Eigen::Vector4d delta_dot_k1 = pseudoInv1 * (-(tau_inp + omega_k1.cross(h1)));
            Eigen::Quaterniond omega_quat_k1(0, omega_k1.x(), omega_k1.y(), omega_k1.z());
            Eigen::Quaterniond att_dot_k1;
            att_dot_k1.coeffs() = (omega_quat_k1 * att_k1).coeffs() * 0.5;
                
            // for RK4 k2 
            Eigen::Vector3d omega_k2 = omega_k1 + 0.5 * Tstep_rk  * omega_dot_k1;
            Eigen::Quaterniond att_k2 = att_k1;
            att_k2.coeffs() += 0.5 * Tstep_rk * att_dot_k1.coeffs();
            Eigen::Vector3d rhs2 = J123 * omega_k2;
            Eigen::Vector3d omega_dot_k2 = J123inv * (tau_allcure - omega_k2.cross(rhs2));
            Eigen::Vector4d delta_k2 = delta_k1 + 0.5 * Tstep_rk * delta_dot_k1;
            Eigen::Matrix<double,4,3> pseudoInv2 = pseudoinverse(delta_k2);
            Eigen::Vector3d h2 = compute_h(delta_k2);
            Eigen::Vector4d delta_dot_k2 = pseudoInv2 * (-(tau_inp + omega_k2.cross(h2)));
            Eigen::Quaterniond omega_quat_k2(0, omega_k2.x(), omega_k2.y(), omega_k2.z());
            Eigen::Quaterniond att_dot_k2;
            att_dot_k2.coeffs() = (omega_quat_k2 * att_k2).coeffs() * 0.5;
               
            // for RK4 k3
            Eigen::Vector3d omega_k3 = omega_k1 + 0.5 * Tstep_rk  * omega_dot_k2;
            Eigen::Quaterniond att_k3 = att_k1;
            att_k3.coeffs() += 0.5 * Tstep_rk * att_dot_k2.coeffs();
            Eigen::Vector3d rhs3 = J123 * omega_k3;
            Eigen::Vector3d omega_dot_k3 = J123inv * (tau_allcure - omega_k3.cross(rhs3));
            Eigen::Vector4d delta_k3 = delta_k1 + 0.5 * Tstep_rk * delta_dot_k2;
            Eigen::Matrix<double,4,3> pseudoInv3 = pseudoinverse(delta_k3);
            Eigen::Vector3d h3 = compute_h(delta_k3);
            Eigen::Vector4d delta_dot_k3 = pseudoInv3 * (-(tau_inp + omega_k3.cross(h3)));
            Eigen::Quaterniond omega_quat_k3(0, omega_k3.x(), omega_k3.y(), omega_k3.z());
            Eigen::Quaterniond att_dot_k3;
            att_dot_k3.coeffs() = (omega_quat_k3 * att_k3).coeffs() * 0.5;
                
            // for RK4 k4
            Eigen::Vector3d omega_k4 = omega_k1 + Tstep_rk * omega_dot_k3;
            Eigen::Quaterniond att_k4 = att_k1;
            att_k4.coeffs() += Tstep_rk * att_dot_k3.coeffs();
            Eigen::Vector3d rhs4 = J123 * omega_k4;
            Eigen::Vector3d omega_dot_k4 = J123inv * (tau_allcure - omega_k4.cross(rhs4));
            Eigen::Vector4d delta_k4 = delta_k1 + Tstep_rk * delta_dot_k3;
            Eigen::Matrix<double,4,3> pseudoInv4 = pseudoinverse(delta_k4);
            Eigen::Vector3d h4 = compute_h(delta_k4);
            Eigen::Vector4d delta_dot_k4 = pseudoInv4 * (-(tau_inp + omega_k4.cross(h4)));
            Eigen::Quaterniond omega_quat_k4(0, omega_k4.x(), omega_k4.y(), omega_k4.z());
            Eigen::Quaterniond att_dot_k4;
            att_dot_k4.coeffs() = (omega_quat_k4 * att_k4).coeffs() * 0.5;
              
            // RK4 
            omega_k1 += (Tstep_rk  / 6.0) * (omega_dot_k1 + 2 * omega_dot_k2 + 2 * omega_dot_k3 + omega_dot_k4);
            delta_k1 += (Tstep_rk  / 6.0) * (delta_dot_k1 + 2 * delta_dot_k2 + 2 * delta_dot_k3 + delta_dot_k4);
            att_k1.coeffs() += (Tstep_rk  / 6.0) * (att_dot_k1.coeffs() + 2 * att_dot_k2.coeffs() + 2 * att_dot_k3.coeffs() + att_dot_k4.coeffs());
            att_k1.normalize();

            omebcur.setValue(omega_k1.x(), omega_k1.y(), omega_k1.z());
            deltacur = delta_k1;
            attcur = att_k1;
                
        }
    }



    void callback_attitude_overwrite(const geometry_msgs::msg::Quaternion::SharedPtr msg){
        //tf2::Quaternion attcur;
        attcur = Eigen::Quaterniond(msg->w, msg->x, msg->y, msg->z);
    }

    void callback_angvel_overwrite(const geometry_msgs::msg::Vector3::SharedPtr msg){
        omebcur.setValue(msg->x,msg->y,msg->z);        
    }

    /**
     * @brief Computes the gravity gradient torque.
     *
    * This function calculates the gravity gradient torque acting on the spacecraft
    * based on its attitude state and inertia properties.
    *
    * @param x The current attitude state.
    * @param par The spacecraft parameters including inertia and gravitational parameters.
    * @return Eigen::Vector3d The gravity gradient torque vector.
    */
    tf2::Vector3 gravityGradT()
    {
        //compute phi(roll) and theta(pitch) from attcur
        double roll, pitch, yaw;

        Eigen::Vector3d angles = attcur.toRotationMatrix().eulerAngles(2,1,0);
        roll = angles(2);
        pitch = angles(1);
        yaw = angles(0);

       // Mean motion calculated from gravitational parameter and orbit radius
        double n = std::sqrt(mu / std::pow(rOrbit, 3));
        double srol = std::sin(roll),  crol = std::cos(roll);
        double spit = std::sin(pitch), cpit = std::cos(pitch);

        double Ixx = J123(0,0);
        double Iyy = J123(1,1);
        double Izz = J123(2,2);
        // Torque components computed based on differences in moments of inertia
        double T1 = (Iyy - Izz) * spit * cpit;
        double T2 = (Izz - Ixx) * srol * crol;
        double T3 = (Ixx - Iyy) * srol * spit;

        return 3.0 * n * n * tf2::Vector3(T1, T2, T3);
    }


    //receiving control torque input
    void callback_cmg_inp(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        tau_ctlcmgcur.setValue(msg->x,msg->y,msg->z); 
        
    }
    
    void callback_bias_inp(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        tau_ctlbiascur.setValue(msg->x,msg->y,msg->z);
    }

    void callback_attitude_dynamics(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        should_pub_att = true;
        tau_ctlthrcur.setValue(msg->x,msg->y,msg->z); 

        //TODO: Currently we assume only gravity gradient torque 
        tau_extgracur = gravityGradT();

        tau_allcur = tau_ctlcmgcur + tau_ctlthrcur + tau_extgracur + tau_ctlbiascur;

        //compute attitude update
        forward_attitude_dynamics(Ttorque_); //Ttorque_ is simulation time period

        //
        i2disp++;
        if(N2disp <= i2disp){
            i2disp = 0;
            RCLCPP_INFO(this->get_logger(), "Received input: %f %f %f", 
                msg->x,msg->y,msg->z);
            RCLCPP_INFO(this->get_logger(), " Angular velocity: %f %f %f (deg/sec)", 
                omebcur.x()/M_PI * 180.0,omebcur.y()/M_PI * 180.0,omebcur.z()/M_PI * 180.0);            
            printq(attcur);
            RCLCPP_INFO(this->get_logger(), " CMG angles: %f %f %f %f(raads)", 
                deltacur(0),deltacur(1),deltacur(2),deltacur(3));
            
        }
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
    auto node = std::make_shared<AttitudeDynamicsNode>(options);
    rclcpp::spin(node);

    rclcpp::shutdown();


    return 0;
}

