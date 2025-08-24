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
#include <random>
#include <Eigen/Dense>

class PhysicsSensor : public rclcpp::Node
{
public:
  PhysicsSensor()
  : Node("physics_sensor")
  {

    // Parameters for sensor noise
    this->declare_parameter("imu_bias_x", 0.0);
    this->declare_parameter("imu_bias_y", 0.0);
    this->declare_parameter("imu_bias_z", 0.0);
    this->declare_parameter("imu_noise_sigma", 0.002);
    this->declare_parameter("bias_drift_sigma", 1e-5);

    this->declare_parameter("startracker_bias_x", 0.0);
    this->declare_parameter("startracker_bias_y", 0.0);
    this->declare_parameter("startracker_bias_z", 0.0);
    this->declare_parameter("startracker_bias_w", 0.0);
    this->declare_parameter("startracker_noise_sigma", 0.001);

    this->declare_parameter("gps_bias_x", 0.0);
    this->declare_parameter("gps_bias_y", 0.0);
    this->declare_parameter("gps_bias_z", 0.0);
    this->declare_parameter("gps_noise_sigma", 0.1);

    // 100Hz subscriber for /gnc/angvel_body
    imu_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/gnc/angvel_body", rclcpp::QoS(10),
      std::bind(&PhysicsSensor::callback_angvel, this, std::placeholders::_1));

    imu_raw_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/gnc/ang_acc", rclcpp::QoS(10),
      std::bind(&PhysicsSensor::callback_angacc, this, std::placeholders::_1));

    // 0.1Hz subscriber for /gnc/attitude_LVLH
    startracker_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
      "/gnc/attitude_LVLH", rclcpp::QoS(10),
      std::bind(&PhysicsSensor::callback_attitude, this, std::placeholders::_1));

    // 1Hz subscriber for /gnc/position_ECEF
    gps_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/gnc/position_ECEF", rclcpp::QoS(10),
      std::bind(&PhysicsSensor::callback_position, this, std::placeholders::_1));

    // Publishers for raw data
    imu_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/gnc/imu_raw", 10);
    startracker_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>(
      "/gnc/startracker_raw", 10);
    gps_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/gnc/gps_raw", 10);

    // Timers to simulate the data processing rates
    // TODO: Require 6DOF output (xddot, yddot, zddot, p, q, r)
    imu_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),       // 100Hz = 10ms interval
      std::bind(&PhysicsSensor::timer_callback_process_imu, this));

    // imu_timer_raw_ = this->create_wall_timer(
    //     std::chrono::milliseconds(10), // 100Hz = 10ms interval
    //     std::bind(&PhysicsSensor::timer_callback_process_imu_raw_, this));

    // TODO: International Celestial Reference Frame (ICRF)
    // TODO: Star tracker algorithm using star catalog and camera images
    startracker_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),       // 0.1Hz = 10 seconds interval TODO: at least this is too fast
      std::bind(&PhysicsSensor::timer_callback_process_startracker, this));

    gps_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),       // 1Hz = 1 second interval
      std::bind(&PhysicsSensor::timer_callback_process_gps, this));

    // Initialize latest sensor data
    bias_drift_x = this->get_parameter("imu_bias_x").as_double();
    bias_drift_y = this->get_parameter("imu_bias_y").as_double();
    bias_drift_z = this->get_parameter("imu_bias_z").as_double();
  }

private:
  std::default_random_engine generator_;

  double generate_noise(double sigma)
  {
    std::normal_distribution<double> distribution(0.0, sigma);
    return distribution(generator_);
  }

  void callback_angvel(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    latest_imu_ = *msg;
  }

  void callback_angacc(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    latest_imu_raw_ = *msg;
  }

  void callback_attitude(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    latest_startracker_ = *msg;
  }

  void callback_position(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    latest_gps_ = *msg;
  }

  void timer_callback_process_imu()
  {
    geometry_msgs::msg::Vector3 noisy_imu_;
    // random walk bias drift
    bias_drift_x += generate_noise(this->get_parameter("bias_drift_sigma").as_double());
    bias_drift_y += generate_noise(this->get_parameter("bias_drift_sigma").as_double());
    bias_drift_z += generate_noise(this->get_parameter("bias_drift_sigma").as_double());

    // accel data with bias drift and noise
    noisy_imu_.x = latest_imu_.x + bias_drift_x +
      generate_noise(this->get_parameter("imu_noise_sigma").as_double());
    noisy_imu_.y = latest_imu_.y + bias_drift_y +
      generate_noise(this->get_parameter("imu_noise_sigma").as_double());
    noisy_imu_.z = latest_imu_.z + bias_drift_z +
      generate_noise(this->get_parameter("imu_noise_sigma").as_double());
    RCLCPP_INFO(
      this->get_logger(), "Processing IMU Data: x=%.3f, y=%.3f, z=%.3f", noisy_imu_.x, noisy_imu_.y,
      noisy_imu_.z);
    imu_pub_->publish(noisy_imu_);
  }

  void timer_callback_process_startracker()
  {
    Eigen::Quaterniond q_true(
      latest_startracker_.w,
      latest_startracker_.x,
      latest_startracker_.y,
      latest_startracker_.z
    );

    // fixed bias vector, small ang axis
    Eigen::Vector3d bias_axis(
      this->get_parameter("startracker_bias_x").as_double(),
      this->get_parameter("startracker_bias_y").as_double(),
      this->get_parameter("startracker_bias_z").as_double()
    );

    // bias_axis.normalized() - axis of rotation
    double bias_angle = bias_axis.norm();
    // constructor Eigen::Quaterniond q(AngleAxisd(..,..));
    Eigen::Quaterniond q_bias = bias_angle >
      1e-12 ? Eigen::Quaterniond(
      Eigen::AngleAxisd(
        bias_angle,
        bias_axis.normalized())) : Eigen::Quaterniond::Identity();

    // noise quat (small random rot vec)
    double noise_sigma = this->get_parameter("startracker_noise_sigma").as_double();
    Eigen::Vector3d noise_vec(
      generate_noise(noise_sigma),
      generate_noise(noise_sigma),
      generate_noise(noise_sigma)
    );

    double noise_angle = noise_vec.norm();
    Eigen::Quaterniond q_noise = noise_angle >
      1e-12 ? Eigen::Quaterniond(
      Eigen::AngleAxisd(
        noise_angle,
        noise_vec.normalized())) : Eigen::Quaterniond::Identity();

    Eigen::Quaterniond q_measured = q_true * q_bias * q_noise;
    q_measured.normalize();

    geometry_msgs::msg::Quaternion noisy_startracker_;
    noisy_startracker_.x = q_measured.x();
    noisy_startracker_.y = q_measured.y();
    noisy_startracker_.z = q_measured.z();
    noisy_startracker_.w = q_measured.w();

    RCLCPP_INFO(
      this->get_logger(),
      "Processing Star Tracker Data: x=%.6f, y=%.6f, z=%.6f, w=%.6f",
      noisy_startracker_.x, noisy_startracker_.y,
      noisy_startracker_.z, noisy_startracker_.w);

    startracker_pub_->publish(noisy_startracker_);
  }

  void timer_callback_process_gps()
  {
    latest_gps_.x += this->get_parameter("gps_bias_x").as_double() + generate_noise(
      this->get_parameter(
        "gps_noise_sigma").as_double());
    latest_gps_.y += this->get_parameter("gps_bias_y").as_double() + generate_noise(
      this->get_parameter(
        "gps_noise_sigma").as_double());
    latest_gps_.z += this->get_parameter("gps_bias_z").as_double() + generate_noise(
      this->get_parameter(
        "gps_noise_sigma").as_double());
    RCLCPP_INFO(
      this->get_logger(), "Processing GPS Data: x=%.3f, y=%.3f, z=%.3f", latest_gps_.x, latest_gps_.y,
      latest_gps_.z);
    gps_pub_->publish(latest_gps_);
  }

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imu_raw_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr startracker_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gps_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr startracker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gps_pub_;

  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr imu_timer_raw_;
  rclcpp::TimerBase::SharedPtr startracker_timer_;
  rclcpp::TimerBase::SharedPtr gps_timer_;

  geometry_msgs::msg::Vector3 latest_imu_;
  geometry_msgs::msg::Vector3 latest_imu_raw_;
  geometry_msgs::msg::Quaternion latest_startracker_;
  geometry_msgs::msg::Vector3 latest_gps_;

  double bias_drift_x = 0.0;
  double bias_drift_y = 0.0;
  double bias_drift_z = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PhysicsSensor>());
  rclcpp::shutdown();
  return 0;
}
