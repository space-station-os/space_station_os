#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

class ControlTorque : public rclcpp::Node {
public:
    ControlTorque()
        : Node("control_torque"), kp_(1.0), kd_(0.1) {
        
        // Declare parameters for PD gains
        this->declare_parameter("kp", 1.0);
        this->declare_parameter("kd", 0.1);

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
    
            pose_ref_.x = 0.0;
            pose_ref_.y = 0.0;
            pose_ref_.z = 0.0;
            pose_ref_.w = 1.0;
    
          
        // Publisher
        torque_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/gnc/torque_cmd", 10);
    }

private:
    void callback_pose_ref(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        pose_ref_ = *msg;
    }

    void callback_pose_est(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        pose_est_ = *msg;
        compute_control();
    }
    
    void callback_angvel_est(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        angvel_est_ = *msg;
    }

    void compute_control() {
        // Load gains from parameters
        kp_ = this->get_parameter("kp").as_double();
        kd_ = this->get_parameter("kd").as_double();

        // Convert quaternions from LVLH to Body frame
        tf2::Quaternion q_ref_LVLH(pose_ref_.x, pose_ref_.y, pose_ref_.z, pose_ref_.w);
        tf2::Quaternion q_est_LVLH(pose_est_.x, pose_est_.y, pose_est_.z, pose_est_.w);

        // Transform from LVLH to Body frame
        tf2::Quaternion q_LVLH_to_Body = q_est_LVLH.inverse();
        tf2::Quaternion q_ref_Body = q_LVLH_to_Body * q_ref_LVLH;

        // Compute quaternion error in body frame
        tf2::Quaternion q_error = q_ref_Body * q_LVLH_to_Body.inverse();
        tf2::Vector3 error_axis(q_error.x(), q_error.y(), q_error.z());

        // Proportional control
        tf2::Vector3 torque_p = kp_ * error_axis;

        // Derivative control using estimated angular velocity
        tf2::Vector3 angvel(angvel_est_.x, angvel_est_.y, angvel_est_.z);
        tf2::Vector3 torque_d = kd_ * (-angvel);

        // Total torque command in body frame
        tf2::Vector3 torque_cmd = torque_p + torque_d;

        // Publish torque command
        geometry_msgs::msg::Vector3 torque_msg;
        torque_msg.x = torque_cmd.x();
        torque_msg.y = torque_cmd.y();
        torque_msg.z = torque_cmd.z();
        torque_pub_->publish(torque_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr pose_ref_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr pose_est_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angvel_est_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr torque_pub_;

    geometry_msgs::msg::Quaternion pose_ref_;
    geometry_msgs::msg::Quaternion pose_est_;
    geometry_msgs::msg::Vector3 angvel_est_;

    double kp_, kd_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlTorque>());
    rclcpp::shutdown();
    return 0;
}
