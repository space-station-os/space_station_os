#ifndef THERMAL_PLUGIN_HH
#define THERMAL_PLUGIN_HH

#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <thread>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport/Node.hh>

#include <thermal_controller/ThermalNodeData.pb.h>
#include <thermal_controller/ThermalLinkFlow.pb.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <space_station_thermal_control/msg/thermal_node_data_array.hpp>
#include <space_station_thermal_control/msg/thermal_link_flows_array.hpp>
#include <space_station_thermal_control/srv/node_heat_flow.hpp>

namespace gz {
namespace sim {
namespace systems {

class ThermalPlugin : public System,
                      public ISystemConfigure,
                      public ISystemPreUpdate
{
public:
  void Configure(const Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &ecm,
                 EventManager &eventMgr) override;

  void PreUpdate(const UpdateInfo &info,
                 EntityComponentManager &ecm) override;

private:
  struct ThermalNode
  {
    std::string jointName;
    std::string linkName;
    double temperature;
    double heatCapacity;
    double internalPower;
  };

  struct ThermalLink
  {
    std::string from;
    std::string to;
    double conductance;
  };

  std::string modelName;
  std::default_random_engine rng;
  std::unordered_map<std::string, ThermalNode> thermalNodes;
  std::vector<ThermalLink> thermalLinks;
  gz::transport::Node gzNode;

  // ROS2
  std::shared_ptr<rclcpp::Node> ros_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread ros_spin_thread_;

  // Publishers
  rclcpp::Publisher<space_station_thermal_control::msg::ThermalNodeDataArray>::SharedPtr ros_node_pub_;
  rclcpp::Publisher<space_station_thermal_control::msg::ThermalLinkFlowsArray>::SharedPtr ros_link_pub_;

  // Cooling service client
  rclcpp::Client<space_station_thermal_control::srv::NodeHeatFlow>::SharedPtr cooling_service_client_;
  bool cooling_active_ = false;
  double cooling_rate_ = 5.0;  // K/sec

  // Stats
  double avg_temperature_ = 0.0;
  double avg_internal_power_ = 0.0;
};

}  // namespace systems
}  // namespace sim
}  // namespace gz

#endif  // THERMAL_PLUGIN_HH
