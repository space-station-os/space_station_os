#ifndef SPACE_STATION_CORE__PLUGIN_BASE_HPP_
#define SPACE_STATION_CORE__PLUGIN_BASE_HPP_

#include <string>
#include <memory>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace space_station_core
{

class PluginBase
{
public:
  using Ptr = std::shared_ptr<PluginBase>;
  virtual ~PluginBase() = default;

  virtual void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
                        const std::string & name) = 0;
  virtual void cleanup() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;
};

}  // namespace space_station_core

#endif  // SPACE_STATION_CORE__PLUGIN_BASE_HPP_
