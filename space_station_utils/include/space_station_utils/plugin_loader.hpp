#ifndef SPACE_STATION_UTILS__PLUGIN_LOADER_HPP_
#define SPACE_STATION_UTILS__PLUGIN_LOADER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <pluginlib/class_loader.hpp>
#include "rclcpp/rclcpp.hpp"

namespace space_station_utils
{

/// Simple wrapper around pluginlib class loader
/// to create plugin instances with basic error handling.
template<class PluginT>
class PluginLoader
{
public:
  /// Package in which the plugins are declared
  explicit PluginLoader(const std::string & package,
                        const std::string & base_class)
  : loader_(package, base_class) {}

  /// Load a plugin instance
  std::shared_ptr<PluginT> createSharedInstance(const std::string & class_name)
  {
    try {
      return loader_.createSharedInstance(class_name);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(rclcpp::get_logger("PluginLoader"),
        "Failed to load plugin %s: %s", class_name.c_str(), ex.what());
    }
    return nullptr;
  }

  const std::vector<std::string> & getDeclaredClasses() const
  {
    return loader_.getDeclaredClasses();
  }

private:
  pluginlib::ClassLoader<PluginT> loader_;
};

}  // namespace space_station_utils

#endif  // SPACE_STATION_UTILS__PLUGIN_LOADER_HPP_
