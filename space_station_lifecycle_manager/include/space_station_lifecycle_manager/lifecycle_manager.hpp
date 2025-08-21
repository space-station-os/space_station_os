#ifndef SPACE_STATION_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define SPACE_STATION_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

namespace space_station_lifecycle_manager
{

class LifecycleManager : public rclcpp::Node
{
public:
  explicit LifecycleManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  bool startup();
  bool shutdown();

private:
  bool changeState(const std::string & node_name, uint8_t transition);
  std::vector<std::string> managed_nodes_;
};

}  // namespace space_station_lifecycle_manager

#endif  // SPACE_STATION_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
