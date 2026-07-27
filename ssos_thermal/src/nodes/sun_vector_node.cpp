#include "ssos_thermal/nodes/sun_vector_node.hpp"

namespace ssos_thermal
{
namespace nodes
{

SunVectorNode::SunVectorNode()
: Node("sun_vector_node")
{
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/gnc/pose_all", 10,
    std::bind(&SunVectorNode::poseCallback, this, std::placeholders::_1));

  sun_vec_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
    "/sun_vector_body", 10);
}

void SunVectorNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  latest_position_ = msg->pose.position;
  latest_quat_ = msg->pose.orientation;
  got_pose_ = true;
  tryComputeSunVector();
}

void SunVectorNode::tryComputeSunVector()
{
  if (!got_pose_) {
    return;
  }

  const rclcpp::Time now = this->now();
  const double jd = sun::computeJulianDate(now);
  const double T = sun::computeJulianCenturies(jd);

  const math3d::Vector3 sun_eci = sun::sunVectorECI(T);

  const math3d::Vector3 r_sc(latest_position_.x, latest_position_.y, latest_position_.z);
  math3d::Vector3 rel_sun_vec = sun_eci - r_sc;
  rel_sun_vec.normalize();

  const math3d::Quaternion q_body(
    latest_quat_.x, latest_quat_.y, latest_quat_.z, latest_quat_.w);
  const math3d::Quaternion q_body_inv = q_body.inverse();

  const math3d::Quaternion s_quat(rel_sun_vec.x, rel_sun_vec.y, rel_sun_vec.z, 0.0);
  const math3d::Quaternion s_body_quat = q_body_inv * s_quat * q_body;

  const math3d::Vector3 s_body(s_body_quat.x, s_body_quat.y, s_body_quat.z);

  geometry_msgs::msg::Vector3 msg_out;
  msg_out.x = s_body.x;
  msg_out.y = s_body.y;
  msg_out.z = s_body.z;

  sun_vec_pub_->publish(msg_out);
}

}  // namespace nodes
}  // namespace ssos_thermal

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ssos_thermal::nodes::SunVectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
