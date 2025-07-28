#include "space_station_thermal_control/sun_vector.hpp"

namespace space_station_thermal_control
{

SunVectorProvider::SunVectorProvider() : Node("sun_vector_provider")
{
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/gnc/pose_all", 10,
        std::bind(&SunVectorProvider::poseCallback, this, std::placeholders::_1));

    sun_vec_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "/sun_vector_body", 10);
}

void SunVectorProvider::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    latest_position_ = msg->pose.position;
    latest_quat_ = msg->pose.orientation;
    got_pose_ = true;
    tryComputeSunVector();
}

void SunVectorProvider::tryComputeSunVector()
{
    if (!got_pose_)
        return;

    rclcpp::Time now = this->now();
    double jd = sun::computeJulianDate(now);
    double T = sun::computeJulianCenturies(jd);

    btVector3 sun_eci = sun::sunVectorECI(T);

    btVector3 r_sc(latest_position_.x, latest_position_.y, latest_position_.z);
    btVector3 rel_sun_vec = sun_eci - r_sc;
    rel_sun_vec.normalize();

    btQuaternion q_body(latest_quat_.x, latest_quat_.y, latest_quat_.z, latest_quat_.w);
    btQuaternion q_body_inv = q_body.inverse();

    btQuaternion s_quat(0.0, rel_sun_vec.x(), rel_sun_vec.y(), rel_sun_vec.z());
    btQuaternion s_body_quat = q_body_inv * s_quat * q_body;

    btVector3 s_body(s_body_quat.x(), s_body_quat.y(), s_body_quat.z());

    geometry_msgs::msg::Vector3 msg_out;
    msg_out.x = s_body.x();
    msg_out.y = s_body.y();
    msg_out.z = s_body.z();

    sun_vec_pub_->publish(msg_out);
}

}  // namespace space_station_thermal_control

int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<space_station_thermal_control::SunVectorProvider>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}