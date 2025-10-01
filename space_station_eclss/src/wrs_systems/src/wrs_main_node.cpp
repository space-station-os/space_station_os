#include "space_station_eclss/wrs_systems.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<space_station_eclss::WRSActionServer>(options);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}