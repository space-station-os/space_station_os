#include "space_station_eclss/ars_systems.hpp"
#include <rclcpp/rclcpp.hpp>


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<space_station_eclss::ARSActionServer>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
