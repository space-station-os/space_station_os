#ifndef SSOS_THERMAL__NODES__SOLAR_HEAT_NODE_HPP_
#define SSOS_THERMAL__NODES__SOLAR_HEAT_NODE_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "space_station_interfaces/msg/solar_panels_q.hpp"

#include "ssos_thermal/math3d.hpp"

// Port of space_station_thermal_control's SolarHeatNode (array_absorptivity
// executable), with btVector3 replaced by ssos_thermal::math3d::Vector3.

namespace ssos_thermal
{
namespace nodes
{

struct PanelParams
{
  double absorptivity;
  double area;
  math3d::Vector3 normal;
};

class SolarHeatNode : public rclcpp::Node
{
public:
  SolarHeatNode();

private:
  void loadPanelParamsFromROS();
  void sunVectorCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  double computePanelHeat(const PanelParams & panel, const math3d::Vector3 & sun_dir);

  double solar_constant_;
  std::unordered_map<std::string, PanelParams> panel_map_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sun_sub_;
  rclcpp::Publisher<space_station_interfaces::msg::SolarPanelsQ>::SharedPtr heat_pub_;
};

}  // namespace nodes
}  // namespace ssos_thermal

#endif  // SSOS_THERMAL__NODES__SOLAR_HEAT_NODE_HPP_
