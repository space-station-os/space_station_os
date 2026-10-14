#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ssos_thermal/nodes/thermal_network_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ssos_thermal::nodes::ThermalNetworkNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
