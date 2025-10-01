#include "space_station_eclss/ogs_system.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc,char **argv){
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<space_station_eclss::OGSSystem>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
