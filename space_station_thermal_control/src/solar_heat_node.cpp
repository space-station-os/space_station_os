#include "space_station_thermal_control/solar_heat_node.hpp"

SolarHeatNode::SolarHeatNode() : Node("solar_heat_node") {
  this->declare_parameter("solar_constant", 1361.0);
  this->declare_parameter("panels_names", std::vector<std::string>{});

  solar_constant_ = this->get_parameter("solar_constant").as_double();
  loadPanelParamsFromROS();

  sun_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "/sun_vector_body", 10,
    std::bind(&SolarHeatNode::sunVectorCallback, this, std::placeholders::_1));

  heat_pub_ = this->create_publisher<space_station_thermal_control::msg::SolarPanelsQ>(
    "/thermal/solar_heat", 10);
}

void SolarHeatNode::loadPanelParamsFromROS() {
  std::vector<std::string> panel_names;
  this->get_parameter("panels_names", panel_names);

  for (const auto &name : panel_names) {
    std::string prefix = name + ".";

    double absorptivity = 0.0;
    double area = 0.0;
    std::vector<double> normal_vec;

    // Declare & get each flattened parameter
    this->declare_parameter(prefix + "absorptivity", 0.0);
    this->declare_parameter(prefix + "area", 0.0);
    this->declare_parameter(prefix + "normal", std::vector<double>{});

    if (!this->get_parameter(prefix + "absorptivity", absorptivity) ||
        !this->get_parameter(prefix + "area", area) ||
        !this->get_parameter(prefix + "normal", normal_vec)) {
      RCLCPP_WARN(this->get_logger(), "Panel %s has missing parameters", name.c_str());
      continue;
    }

    if (normal_vec.size() != 3) {
      RCLCPP_WARN(this->get_logger(), "Panel %s has invalid normal vector", name.c_str());
      continue;
    }

    PanelParams p;
    p.absorptivity = absorptivity;
    p.area = area;
    p.normal = btVector3(normal_vec[0], normal_vec[1], normal_vec[2]).normalized();

    panel_map_[name] = p;
    RCLCPP_INFO(this->get_logger(), "Loaded panel %s: alpha=%.2f, area=%.2f", name.c_str(), p.absorptivity, p.area);
  }

  RCLCPP_INFO(this->get_logger(), "[SolarHeatNode] Total panels loaded: %zu", panel_map_.size());
}

void SolarHeatNode::sunVectorCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  btVector3 sun_dir(msg->x, msg->y, msg->z);
  sun_dir.normalize();

  space_station_thermal_control::msg::SolarPanelsQ out_msg;
  out_msg.header.stamp = this->now();

  for (const auto &[name, panel] : panel_map_) {
    double Q = computePanelHeat(panel, sun_dir);
    out_msg.names.push_back(name);
    out_msg.heat_watts.push_back(Q);
  }

  heat_pub_->publish(out_msg);
}

double SolarHeatNode::computePanelHeat(const PanelParams &panel, const btVector3 &sun_dir) {
  double cos_theta = panel.normal.dot(sun_dir);
  cos_theta = std::max(0.0, cos_theta);
  RCLCPP_INFO(this->get_logger(), "Theta (deg): %.2f", std::acos(cos_theta) * 180.0 / M_PI);
  return panel.absorptivity * panel.area * solar_constant_ * cos_theta;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SolarHeatNode>());
  rclcpp::shutdown();
  return 0;
}
