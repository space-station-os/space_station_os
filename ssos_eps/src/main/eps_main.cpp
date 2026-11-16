#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ssos_eps/nodes/eps_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ssos_eps::nodes::EpsNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
