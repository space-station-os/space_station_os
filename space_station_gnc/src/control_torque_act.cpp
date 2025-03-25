#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <Eigen/Dense>

class ControlTorque : public rclcpp::Node {
public:
    ControlTorque()
        : Node("control_torque"), kp_(300000.0), kd_(300000) {
        
        // Declare parameters for PD gains
        this->declare_parameter("kp", 300000.0);
        this->declare_parameter("kd", 300000.0);

        // Subscribers
        pose_ref_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/gnc/pose_ref", rclcpp::QoS(10),
            std::bind(&ControlTorque::callback_pose_ref, this, std::placeholders::_1));
        
        pose_act_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "gnc/attitude_LVLH", rclcpp::QoS(10),
            std::bind(&ControlTorque::callback_pose_act, this, std::placeholders::_1));
            
        angvel_act_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "gnc/angvel_body", rclcpp::QoS(10),
            std::bind(&ControlTorque::callback_angvel_act, this, std::placeholders::_1));
    
            pose_ref_.x = 0.0;
            pose_ref_.y = 0.0;
            pose_ref_.z = 0.0;
            pose_ref_.w = 1.0;
    
          
        // Publisher
        torque_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/gnc/cmg_torque_cmd", 10);
    }

private:
    void callback_pose_ref(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        pose_ref_ = *msg;
    }

    void callback_pose_act(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        pose_act_ = *msg;
        compute_control();
    }
    
    void callback_angvel_act(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        angvel_act_ = *msg;
    }

    void compute_control() {
        // Load gains from parameters
        kp_ = this->get_parameter("kp").as_double();
        kd_ = this->get_parameter("kd").as_double();

        // Convert quaternions from LVLH to Body frame
        // tf2::Quaternion q_ref_LVLH(pose_ref_.x, pose_ref_.y, pose_ref_.z, pose_ref_.w);
        Eigen::Quaterniond q_ref_LVLH = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond q_act_LVLH(pose_act_.w, pose_act_.x, pose_act_.y, pose_act_.z);
        q_act_LVLH.normalize();
        // Transform from LVLH to Body frame
        // tf2::Quaternion q_LVLH_to_Body = q_act_LVLH.inverse();
        // tf2::Quaternion q_ref_Body = q_LVLH_to_Body * q_ref_LVLH;

        // Compute quaternion error in body frame
        Eigen::Quaterniond q_error = q_act_LVLH.conjugate() * q_ref_LVLH;

        if (q_error.w() < 0) {
            q_error.coeffs() *= -1;
        }

        double theta = 2.0 * std::acos(q_error.w());
        double sin_half_theta = std::sqrt(1- q_error.w() * q_error.w());

        Eigen::Vector3d error_axis_norm;
        if (sin_half_theta > 1e-6) {
            error_axis_norm = q_error.vec() / sin_half_theta;

        } else{
            error_axis_norm = Eigen::Vector3d::Zero();
        }
        Eigen::Vector3d error_vector = theta * error_axis_norm;
        // Proportional control
        Eigen::Vector3d torque_p = kp_ * error_vector;

        // Derivative control using estimated angular velocity
        Eigen::Vector3d angvel(angvel_act_.x, angvel_act_.y, angvel_act_.z);
        Eigen::Vector3d torque_d = kd_ * (-angvel);

        // Total torque command in body frame
        Eigen::Vector3d torque_cmd = torque_p + torque_d;

        // Publish torque command
        geometry_msgs::msg::Vector3 torque_msg;
        torque_msg.x = torque_cmd.x();
        torque_msg.y = torque_cmd.y();
        torque_msg.z = torque_cmd.z();
        torque_pub_->publish(torque_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr pose_ref_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr pose_act_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angvel_act_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr torque_pub_;

    geometry_msgs::msg::Quaternion pose_ref_;
    geometry_msgs::msg::Quaternion pose_act_;
    geometry_msgs::msg::Vector3 angvel_act_;

    double kp_, kd_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlTorque>());
    rclcpp::shutdown();
    return 0;
}
