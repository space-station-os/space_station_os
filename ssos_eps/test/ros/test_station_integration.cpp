#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "space_station_interfaces/msg/load_status_array.hpp"
#include "space_station_interfaces/msg/system_state.hpp"
#include "space_station_interfaces/msg/thermal_node_data_array.hpp"
#include "ssos_core/system_manager.hpp"
#include "space_station_interfaces/srv/inject_fault.hpp"
#include "ssos_eclss/nodes/ars_node.hpp"
#include "ssos_eclss/nodes/cabin_node.hpp"
#include "ssos_eclss/nodes/crew_node.hpp"
#include "ssos_eclss/nodes/ogs_node.hpp"
#include "ssos_eclss/nodes/sabatier_node.hpp"
#include "ssos_eclss/nodes/wrs_node.hpp"
#include "ssos_sim/simulation_controller.hpp"
#include "ssos_thermal/nodes/thermal_network_node.hpp"
#include "std_msgs/msg/float64.hpp"

#include "ssos_eps/nodes/eps_node.hpp"

// Whole station in one process: simulation_controller drives sim time,
// system_manager supervises, the crew and cabin load the ECLSS units, which
// draw their power from eps_node, and EPS heat goes into the thermal network
// whose cold plate sets the EPS hardware temperature. The station must start
// NOMINAL; a drill that trips the DDCU feeding OGS stops oxygen production,
// and clearing it returns the station to NOMINAL.

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using space_station_interfaces::msg::LoadStatus;
using space_station_interfaces::msg::LoadStatusArray;
using space_station_interfaces::msg::SystemState;
using space_station_interfaces::msg::ThermalNodeDataArray;
using space_station_interfaces::srv::InjectFault;

namespace
{
const std::vector<std::string> kEclssLoads = {
  "eclss_ars", "eclss_ogs", "eclss_wrs", "eclss_sabatier"};
}

class StationIntegrationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  template<typename NodeT>
  std::shared_ptr<NodeT> make(std::vector<rclcpp::Parameter> params = {})
  {
    params.emplace_back("use_sim_time", true);
    auto node = std::make_shared<NodeT>(rclcpp::NodeOptions().parameter_overrides(params));
    executor_.add_node(node->get_node_base_interface());
    lifecycle_.push_back(node);
    return node;
  }

  void SetUp() override
  {
    manager_ = std::make_shared<ssos_core::SystemManager>(
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("recovery_hold_s", 1.0),
        rclcpp::Parameter("eval_period_s", 0.2)}));
    executor_.add_node(manager_->get_node_base_interface());
    sim_ = std::make_shared<ssos_sim::SimulationController>(
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("sim_rate_hz", 20.0),
        rclcpp::Parameter("time_scale", 20.0),
        rclcpp::Parameter("sim_duration_s", 1.0e9)}));
    executor_.add_node(sim_->get_node_base_interface());

    eps_ = make<ssos_eps::nodes::EpsNode>({rclcpp::Parameter("step_rate_hz", 20.0)});
    thermal_ = make<ssos_thermal::nodes::ThermalNetworkNode>({
        rclcpp::Parameter("thermal_config_file", std::string(SSOS_THERMAL_STATION_CONFIG))});
    const std::vector<rclcpp::Parameter> eclss_rate = {rclcpp::Parameter("step_rate_hz", 5.0)};
    make<ssos_eclss::nodes::CrewNode>(eclss_rate);
    make<ssos_eclss::nodes::CabinNode>(eclss_rate);
    ars_ = make<ssos_eclss::nodes::ArsNode>(eclss_rate);
    ogs_ = make<ssos_eclss::nodes::OgsNode>(eclss_rate);
    wrs_ = make<ssos_eclss::nodes::WrsNode>(eclss_rate);
    sabatier_ = make<ssos_eclss::nodes::SabatierNode>(eclss_rate);

    probe_ = std::make_shared<rclcpp::Node>("station_probe");
    executor_.add_node(probe_);
    state_sub_ = probe_->create_subscription<SystemState>(
      "/ssos/system_state", 10, [this](SystemState::SharedPtr m) {states_.push_back(m->state);});
    loads_sub_ = probe_->create_subscription<LoadStatusArray>(
      "/ssos/eps/loads", 10, [this](LoadStatusArray::SharedPtr m) {
        for (const auto & l : m->loads) {
          loads_[l.load_id] = l;
        }
      });
    thermal_sub_ = probe_->create_subscription<ThermalNodeDataArray>(
      "/thermal/nodes/state", 10, [this](ThermalNodeDataArray::SharedPtr m) {
        for (const auto & n : m->nodes) {
          thermal_power_[n.name] = n.internal_power;
          thermal_temp_[n.name] = n.temperature;
        }
      });
    inject_client_ = probe_->create_client<InjectFault>("/sim/inject_fault");
    o2_sub_ = probe_->create_subscription<std_msgs::msg::Float64>(
      "/ssos/ogs/o2_kg_day", 10, [this](std_msgs::msg::Float64::SharedPtr m) {o2_ = m->data;});

    ASSERT_EQ(manager_->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(manager_->activate().id(), State::PRIMARY_STATE_ACTIVE);
    for (auto & node : lifecycle_) {
      ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE) << node->get_name();
      ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE) << node->get_name();
    }
    ASSERT_EQ(sim_->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(sim_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  }

  void TearDown() override
  {
    executor_.remove_node(probe_);
    for (auto & node : lifecycle_) {
      executor_.remove_node(node->get_node_base_interface());
    }
    executor_.remove_node(sim_->get_node_base_interface());
    executor_.remove_node(manager_->get_node_base_interface());
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end && rclcpp::ok()) {
      executor_.spin_some();
      std::this_thread::sleep_for(2ms);
    }
  }

  bool inject(const std::string & target, const std::string & type, const std::string & json)
  {
    auto req = std::make_shared<InjectFault::Request>();
    req->target_subsystem = target;
    req->fault_type = type;
    req->parameters_json = json;
    req->duration_s = 0.0;
    auto future = inject_client_->async_send_request(req);
    const auto end = std::chrono::steady_clock::now() + 2s;
    while (future.wait_for(0s) != std::future_status::ready &&
      std::chrono::steady_clock::now() < end)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(2ms);
    }
    return future.wait_for(0s) == std::future_status::ready && future.get()->success;
  }

  bool saw_in_order(const std::vector<uint8_t> & expected) const
  {
    auto it = states_.begin();
    for (auto s : expected) {
      it = std::find(it, states_.end(), s);
      if (it == states_.end()) {
        return false;
      }
      ++it;
    }
    return true;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<ssos_core::SystemManager> manager_;
  std::shared_ptr<ssos_sim::SimulationController> sim_;
  std::shared_ptr<ssos_eps::nodes::EpsNode> eps_;
  std::shared_ptr<ssos_thermal::nodes::ThermalNetworkNode> thermal_;
  std::shared_ptr<ssos_eclss::nodes::ArsNode> ars_;
  std::shared_ptr<ssos_eclss::nodes::OgsNode> ogs_;
  std::shared_ptr<ssos_eclss::nodes::WrsNode> wrs_;
  std::shared_ptr<ssos_eclss::nodes::SabatierNode> sabatier_;
  std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> lifecycle_;

  rclcpp::Node::SharedPtr probe_;
  rclcpp::Subscription<SystemState>::SharedPtr state_sub_;
  rclcpp::Subscription<LoadStatusArray>::SharedPtr loads_sub_;
  rclcpp::Subscription<ThermalNodeDataArray>::SharedPtr thermal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr o2_sub_;
  rclcpp::Client<InjectFault>::SharedPtr inject_client_;
  std::vector<uint8_t> states_;
  std::map<std::string, LoadStatus> loads_;
  std::map<std::string, double> thermal_power_;
  std::map<std::string, double> thermal_temp_;
  double o2_{-1.0};
};

TEST_F(StationIntegrationTest, PowerThermalAndLifeSupportCloseTheLoop)
{
  // Sim runs 20x: 1.5 s wall is 30 s of station time.
  spin_for(1500ms);

  // ECLSS registered its own loads with EPS, all powered.
  for (const auto & id : kEclssLoads) {
    ASSERT_TRUE(loads_.count(id)) << id;
    EXPECT_FALSE(loads_[id].shed) << id;
    EXPECT_GT(loads_[id].served_power, 0.0) << id;
  }
  // Sabatier is not a baseline load: it can only be there by request.
  EXPECT_GT(loads_["eclss_sabatier"].requested_power, 0.0);
  EXPECT_GT(o2_, 1.0);

  // EPS heat reaches the thermal network, and EPS reads the cold plate back.
  ASSERT_TRUE(thermal_power_.count("itcs_coldplate"));
  EXPECT_GT(thermal_power_["itcs_coldplate"], 100.0);
  EXPECT_GT(thermal_power_["base_link"], 1380.0 + 10000.0);
  EXPECT_TRUE(std::isfinite(eps_->coldplate_temp_c()));
  EXPECT_NEAR(eps_->coldplate_temp_c(), thermal_temp_["itcs_coldplate"], 0.5);
  EXPECT_GT(eps_->coldplate_temp_c(), 17.0);

  // Already on orbit: straight to NOMINAL, never DEGRADED on the way.
  EXPECT_EQ(manager_->get_current_state(), SystemState::NOMINAL);
  EXPECT_EQ(std::count(states_.begin(), states_.end(), SystemState::DEGRADED), 0);

  // Drill: trip the DDCU feeding OGS. Its load is dropped, oxygen production
  // stops and the station goes DEGRADED.
  const int ddcu = loads_["eclss_ogs"].ddcu_index;
  ASSERT_GE(ddcu, 0);
  ASSERT_TRUE(inject("eps", "ddcu_fault", "{\"ddcu\": " + std::to_string(ddcu) + "}"));
  spin_for(1500ms);
  EXPECT_TRUE(eps_->last_state().ddcu_fault);
  EXPECT_TRUE(loads_["eclss_ogs"].shed);
  EXPECT_DOUBLE_EQ(o2_, 0.0);
  EXPECT_EQ(manager_->get_current_state(), SystemState::DEGRADED);
  // One drill at a time.
  EXPECT_FALSE(inject("ogs", "ogs_generation_failure", "{}"));

  // Drill cleared: the DDCU comes back, OGS is powered again and the station
  // recovers.
  ASSERT_TRUE(inject("eps", "clear_faults", "{}"));
  spin_for(3000ms);
  EXPECT_FALSE(eps_->last_state().ddcu_fault);
  EXPECT_FALSE(loads_["eclss_ogs"].shed);
  EXPECT_GT(o2_, 1.0);
  EXPECT_EQ(manager_->get_current_state(), SystemState::NOMINAL);
  EXPECT_TRUE(saw_in_order({SystemState::NOMINAL, SystemState::DEGRADED,
      SystemState::RECOVERY, SystemState::NOMINAL}));
}
