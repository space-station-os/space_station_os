#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/empty.hpp>
#include <rclcpp/rclcpp.hpp>

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
  }

private:
  void on_unload(const std_msgs::msg::Empty::SharedPtr)
  {
    RCLCPP_WARN(this->get_logger(), "CMG unloading triggered");
    unloading_ = true;
  }

  void on_cmd(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    geometry_msgs::msg::Vector3 out = *msg;
    if (unloading_) {
      out.x = out.y = out.z = 0.0;
      unloading_ = false;  // one-shot
    }
    pub_cmg_->publish(out);
  }

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_unload_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_cmg_;
  bool unloading_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorqueController>());
  rclcpp::shutdown();
  return 0;
}

