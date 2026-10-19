#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ssos_eps/eps_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<ssos_eps::EpsNode>();

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(
    node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
