#include "space_station_bt_navigator/bt_navigator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "space_station_lifecycle_manager/lifecycle_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto navigator = std::make_shared<space_station_bt_navigator::BtNavigator>();
  auto manager = std::make_shared<space_station_lifecycle_manager::LifecycleManager>();
  manager->set_parameters({rclcpp::Parameter("managed_nodes", std::vector<std::string>{navigator->get_name()})});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(navigator);
  exec.add_node(manager);

  manager->startup();
  navigator->execute();
  manager->shutdown();
  rclcpp::shutdown();
  return 0;
}
