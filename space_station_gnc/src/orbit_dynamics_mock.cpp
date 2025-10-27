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


// This Orbit dynamics mock will be replaced with the GNC dashboard that would subscribe to the below mentioned topics

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class OrbitDynamicsMockNode : public rclcpp::Node
{
public:
  OrbitDynamicsMockNode()
  : Node("orbit_dynamics_mock_node")
  {
    pub_t_fwd_sim_ = this->create_publisher<std_msgs::msg::Float64>("gnc/t_fwd_sim", 10);
    pub_attitude_ = this->create_publisher<geometry_msgs::msg::Quaternion>(
      "gnc/attitude_body_to_eci", 10);
    pub_thruster_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "gnc/bias_thruster_cmd", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&OrbitDynamicsMockNode::publish_mock_data, this)
    );

    RCLCPP_INFO(this->get_logger(), "OrbitDynamicsMockNode started.");
  }

private:
  void publish_mock_data()
  {
    // t_fwd_sim
    std_msgs::msg::Float64 t_fwd_sim_msg;
    t_fwd_sim_msg.data = 60.0;
    pub_t_fwd_sim_->publish(t_fwd_sim_msg);
    RCLCPP_INFO(this->get_logger(), "Published: gnc/t_fwd_sim");

    // attitude
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg.x = 0.0;
    quat_msg.y = 0.0;
    quat_msg.z = 0.0;
    quat_msg.w = 1.0;
    pub_attitude_->publish(quat_msg);
    RCLCPP_INFO(this->get_logger(), "Published: gnc/attitude_body_to_eci");

    // bias_thruster_cmd
    std_msgs::msg::Float32MultiArray thruster_msg;
    thruster_msg.data.resize(12, 0.0f);
    pub_thruster_->publish(thruster_msg);

    RCLCPP_INFO(this->get_logger(), "Published: gnc/bias_thruster_cmd");
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_t_fwd_sim_;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_attitude_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_thruster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OrbitDynamicsMockNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
