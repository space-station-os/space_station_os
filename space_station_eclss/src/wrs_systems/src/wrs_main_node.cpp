#include "space_station_eclss/wrs_systems.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<space_station_eclss::WRSActionServer>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}