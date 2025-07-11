#ifndef SOLAR_HEAT_NODE_HPP_
#define SOLAR_HEAT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <space_station_thermal_control/msg/solar_panels_q.hpp>

#include <unordered_map>
#include <string>
#include <vector>
#include <bullet/LinearMath/btVector3.h>

struct PanelParams {
  double absorptivity;
  double area;
  btVector3 normal;
};

class SolarHeatNode : public rclcpp::Node {
public:
  SolarHeatNode();

private:
  void loadPanelParamsFromROS();
  void sunVectorCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  double computePanelHeat(const PanelParams &panel, const btVector3 &sun_dir);

  double solar_constant_;
  std::unordered_map<std::string, PanelParams> panel_map_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sun_sub_;
  rclcpp::Publisher<space_station_thermal_control::msg::SolarPanelsQ>::SharedPtr heat_pub_;
};

#endif  // SOLAR_HEAT_NODE_HPP_
