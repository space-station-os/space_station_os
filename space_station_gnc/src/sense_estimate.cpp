// Copyright 2025 Space Station OS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

// NO sensor estimation algorithms are implemented in this file.Rather just republishing
class SenseEstimate : public rclcpp::Node
{
public:
  SenseEstimate()
  : Node("sense_estimate")
  {

    // Subscribers for raw sensor data
    imu_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/gnc/imu_raw", rclcpp::QoS(10),
      std::bind(&SenseEstimate::callback_imu, this, std::placeholders::_1));

    startracker_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
      "/gnc/startracker_raw", rclcpp::QoS(10),
      std::bind(&SenseEstimate::callback_startracker, this, std::placeholders::_1));

    gps_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/gnc/gps_raw", rclcpp::QoS(10),
      std::bind(&SenseEstimate::callback_gps, this, std::placeholders::_1));

    // Publishers for estimated data
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/gnc/pose_est", 10);
    angvel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/gnc/angvel_est", 10);
    position_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/gnc/position_est", 10);
  }

private:
  void callback_imu(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    latest_imu_ = *msg;
    angvel_pub_->publish(latest_imu_);
  }

  void callback_startracker(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    latest_attitude_ = *msg;
    pose_pub_->publish(latest_attitude_);
  }

  void callback_gps(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    latest_position_ = *msg;
    position_pub_->publish(latest_position_);
  }

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr startracker_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gps_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angvel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_pub_;

  geometry_msgs::msg::Vector3 latest_imu_;
  geometry_msgs::msg::Quaternion latest_attitude_;
  geometry_msgs::msg::Vector3 latest_position_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SenseEstimate>());
  rclcpp::shutdown();
  return 0;
}
