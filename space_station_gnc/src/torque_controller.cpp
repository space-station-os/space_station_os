#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

class TorqueController : public rclcpp::Node
{
public:
  TorqueController()
  : Node("torque_controller"), unloading_(false)
  {
    sub_cmd_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "gnc/torque_cmd", 10,
      std::bind(&TorqueController::on_cmd, this, std::placeholders::_1));
    sub_unload_ = this->create_subscription<std_msgs::msg::Empty>(
      "gnc/unload_cmg", 10,
      std::bind(&TorqueController::on_unload, this, std::placeholders::_1));
    pub_cmg_ = this->create_publisher<geometry_msgs::msg::Vector3>(
      "gnc/cmg_torque_cmd", 10);
    pub_status_ = this->create_publisher<std_msgs::msg::String>(
      "gnc/cmg_status", 10);
    // Create a timer to periodically publish CMG status
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TorqueController::publish_status, this));
  }

private:
  void publish_status()
  {
    std_msgs::msg::String status_msg;
    if (unloading_) {
      status_msg.data = "unloading";
    } else {
      status_msg.data = "idle";
    }
    pub_status_->publish(status_msg);
  }

  void on_unload(const std_msgs::msg::Empty::SharedPtr)
  {
    RCLCPP_WARN(this->get_logger(), "CMG unloading triggered");
    unloading_ = true;
    
    // Publish status update
    std_msgs::msg::String status_msg;
    status_msg.data = "unloading";
    pub_status_->publish(status_msg);
  }

  void on_cmd(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    geometry_msgs::msg::Vector3 out = *msg;
    if (unloading_) {
      out.x = out.y = out.z = 0.0;
      unloading_ = false;  // one-shot
      
      // Publish status update when unloading is complete
      std_msgs::msg::String status_msg;
      status_msg.data = "unloaded";
      pub_status_->publish(status_msg);
    }
    pub_cmg_->publish(out);
  }

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_unload_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_cmg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  bool unloading_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorqueController>());
  rclcpp::shutdown();
  return 0;
}

