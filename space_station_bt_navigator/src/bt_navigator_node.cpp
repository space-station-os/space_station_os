#include "space_station_bt_navigator/bt_navigator.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<space_station_bt_navigator::BtNavigator>();
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
