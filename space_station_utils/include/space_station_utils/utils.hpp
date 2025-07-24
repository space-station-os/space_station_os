#ifndef SPACE_STATION_UTILS__UTILS_HPP_
#define SPACE_STATION_UTILS__UTILS_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"

namespace space_station_utils
{

/// Declare parameter if not already declared and return its value
template<typename NodeT, typename ParameterT>
ParameterT
get_parameter_or(NodeT node, const std::string & name, const ParameterT & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, default_value);
  }
  return node->get_parameter(name).template get_value<ParameterT>();
}

}  // namespace space_station_utils

#endif  // SPACE_STATION_UTILS__UTILS_HPP_
