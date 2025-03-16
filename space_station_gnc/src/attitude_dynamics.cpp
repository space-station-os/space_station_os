#include <rclcpp/rclcpp.hpp>
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
#include <chrono>
#include <thread>
//std::string mode_demo;


class AttitudeDynamicsNode : public rclcpp::Node
{
public:
    AttitudeDynamicsNode() : Node("attitude_dynamics_node")
    {
        pub_attitude_all = this->create_publisher<geometry_msgs::msg::Quaternion>("attitude_all", 1);
        pub_angvel_body = this->create_publisher<geometry_msgs::msg::Vector3>("angvel_body", 1);
        pub_pose_all = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_all", 10);
        //pub_pose_marker = this->create_publisher<visualization_msgs::msg::Marker>("pose_marker", 10);
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);


        i2disp = 0; //publish rate: Npublish * Ttorque
        should_pub_att = false;

        omebcur.setValue(0.0,0.0,0.0);
        //omebcur.setValue(0.0,+0.22 / 180.0 * M_PI,0.0);
        omebprv = omebcur;
        omedotbcur.setValue(0.0,0.0,0.0);
        //omedotbcur.setValue(0.0,+0.0003 / 180.0 * M_PI,0.0);
        omedotbcur.setValue(0.0,+0.0000780,0.0);
        omedotbprv = omedotbcur;
        attcur.setRPY(0.0,0.0,0.0);
        publish_attitude(attcur);
        printq(attcur);
        
        //attcur.setRPY(0.0,1.72/180.0*M_PI,0.0);
        //attcur.setRPY(0.0,11.97/180.0*M_PI,0.0);
        attprv = attcur;
        
        double J11 =  280.0 * 1000.0 * 1000.0;
        double J22 =  140.0 * 1000.0 * 1000.0;
        double J33 =  420.0 * 1000.0 * 1000.0;
        J123.setValue(
             J11, 0.0, 0.0,
             0.0, J22, 0.0,
             0.0, 0.0, J33);
        J123inv = J123.inverse();

        sub_t_fwd_sim = this->create_subscription<std_msgs::msg::Float64>(
            "t_fwd_sim", 1, 
            std::bind(&AttitudeDynamicsNode::callback_t_fwd_sim, this, std::placeholders::_1));

        sub_attitude_overwrite = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "attitude_overwrite", 1, 
            std::bind(&AttitudeDynamicsNode::callback_attitude_overwrite, this, std::placeholders::_1));

        sub_angvel_overwrite = this->create_subscription<geometry_msgs::msg::Vector3>(
            "angvel_overwrite", 1, 
            std::bind(&AttitudeDynamicsNode::callback_angvel_overwrite, this, std::placeholders::_1));

        sub_torque_control = this->create_subscription<geometry_msgs::msg::Vector3>(
            "torque_control", 1, 
            std::bind(&AttitudeDynamicsNode::callback_attitude_dynamics, this, std::placeholders::_1));
            
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(Tpubatt_*1000.0)), 
            std::bind(&AttitudeDynamicsNode::callback_timer_pub_att, this));


    }

private:
    
    //ros2 stuff
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_torque_control; 
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_t_fwd_sim;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_angvel_overwrite;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_attitude_overwrite;

    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_attitude_all;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_angvel_body;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_all;
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_pose_marker;
    geometry_msgs::msg::Quaternion attitude_all;
    geometry_msgs::msg::Vector3 angvel_body;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;


    //define parameters for dynamics
    //TODO: Jall, dynamic change of J, 
    // as well as CoG offset issue is a big TODO
    tf2::Matrix3x3 J123;
    tf2::Matrix3x3 J123inv;

    tf2::Quaternion attcur; // pose
    tf2::Quaternion attprv; // pose
    tf2::Vector3 omebcur; //rad/sec, angular velocity of body frame
    tf2::Vector3 omebprv; //rad/sec, angular velocity of body frame
    tf2::Vector3 omedotbcur; //rad/sec^2 angular acc of body frame
    tf2::Vector3 omedotbprv; //rad/sec^2 angular acc of body frame
    
    tf2::Vector3 tau_allcur; 
    tf2::Vector3 tau_allprv; 



    //define parameters for simulation
    const double Tstep_ = 0.001; // in seconds
    const double Ttorque_ = 0.1; //same as T_callback
    //int Nstep; // number of steps for simulation, Ttorque_/Tstep_

    const int N2disp = 10; // publish rate: Npublish * Ttorque
    int i2disp; // index for i2disp
    const double Tpubatt_ = 0.1; //same as T_callback

    bool should_pub_att; // true then publish attitude for display purpose


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

    //utility function
    void printq(const tf2::Quaternion& q){
        tf2::Matrix3x3 m(q); //r11,r21,r31 show where original x axis points
        double r11 = m[0][0], r12 = m[0][1], r13 = m[0][2];
        double r21 = m[1][0], r22 = m[1][1], r23 = m[1][2];
        double r31 = m[2][0], r32 = m[2][1], r33 = m[2][2];
        RCLCPP_INFO(this->get_logger(),"  [%+.4f, %+.4f, %+.4f]",r11,r12,r13);
        RCLCPP_INFO(this->get_logger(),"  [%+.4f, %+.4f, %+.4f]",r21,r22,r23);
        RCLCPP_INFO(this->get_logger(),"  [%+.4f, %+.4f, %+.4f]",r31,r32,r33);
    }

    //utility function
    void publish_attitude(tf2::Quaternion att){
        attitude_all.x = att.x();
        attitude_all.y = att.y();
        attitude_all.z = att.z();
        attitude_all.w = att.w();
        pub_attitude_all->publish(attitude_all);   //rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_attitude_all;
            
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

    void callback_timer_pub_att() {
        if(!should_pub_att)
            return;
        publish_attitude(attcur);

        //
        angvel_body.x = omebcur.x();
        angvel_body.y = omebcur.y();
        angvel_body.z = omebcur.z();
        pub_angvel_body->publish(angvel_body);           
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



    void forward_attitude_dynamics(double Tfwd_sec){
        //int Nstep = (int)(Tfwd_sec/Tstep_);
        int Nstep = (int)(Tfwd_sec/Tstep_);

        for(int istep = 0; istep < Nstep; istep++){
            omebprv = omebcur;
            omedotbprv = omedotbcur; //omedotbprv not used
            attprv = attcur;

            //1. compute omedotbcur: 
            // J123 * omedotbcur = torque - omebprv (cross) (J123*omebprv)
            tf2::Vector3 rhs1 = J123 * omebprv;
            tf2::Vector3 rhs2 = tau_allcur - omebprv.cross(rhs1);
            omedotbcur = J123inv * rhs2;

            //2. compute omebcur
            omebcur = omebprv + omedotbcur * Tstep_;
            
            //3. compute moved angle 
            attcur = stepupdate_attitude(attprv,omebcur,Tstep_);
        }


    }


    void callback_attitude_overwrite(const geometry_msgs::msg::Quaternion::SharedPtr msg){
        //tf2::Quaternion attcur;
        attcur.setValue(msg->x, msg->y, msg->z, msg->w);
    }

    void callback_angvel_overwrite(const geometry_msgs::msg::Vector3::SharedPtr msg){
        omebcur.setValue(msg->x,msg->y,msg->z);        
    }

    //receiving control torque input
    void callback_attitude_dynamics(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        should_pub_att = true;
        tau_allprv = tau_allcur;
        tau_allcur.setValue(msg->x,msg->y,msg->z); 
        
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
    auto node = std::make_shared<AttitudeDynamicsNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();


    return 0;
}

