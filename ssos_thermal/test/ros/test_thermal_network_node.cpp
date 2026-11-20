#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "space_station_interfaces/msg/heat_load_array.hpp"
#include "space_station_interfaces/msg/thermal_link_flows_array.hpp"
#include "space_station_interfaces/msg/thermal_node_data_array.hpp"
#include "ssos_sim/fault_format.hpp"

#include "ssos_thermal/nodes/thermal_network_node.hpp"

using namespace ssos_thermal::nodes;
using namespace std::chrono_literals;

namespace
{

void spin_node_for(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & base,
  std::chrono::milliseconds d)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(base);
  const auto end = std::chrono::steady_clock::now() + d;
  while (std::chrono::steady_clock::now() < end && rclcpp::ok()) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }
}

// Load the graph from the source tree so the test does not depend on
// install/ssos_thermal being on AMENT_PREFIX_PATH.
rclcpp::Parameter source_config(const std::string & file = "thermal_nodes.yaml")
{
  return rclcpp::Parameter("thermal_config_file",
                           std::string(SSOS_THERMAL_SOURCE_DIR) + "/config/" + file);
}

bool spin_until(rclcpp::Executor & exec, const std::function<bool()> & done,
                std::chrono::milliseconds timeout)
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (!done() && std::chrono::steady_clock::now() < end && rclcpp::ok()) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  return done();
}

// Plays the simulation and the rest of the station: injects drills, heat
// loads and /clock, and records what thermal_network publishes.
struct Probe
{
  Probe()
  : node(std::make_shared<rclcpp::Node>("thermal_test_probe"))
  {
    using namespace space_station_interfaces::msg;
    fault_pub = node->create_publisher<FaultEvent>("/ssos/fault_event", 10);
    heat_pub = node->create_publisher<HeatLoadArray>("/thermal/heat_loads", 10);
    clock_pub = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    subs.push_back(node->create_subscription<ThermalNodeDataArray>(
        "/thermal/nodes/state", 10, [this](ThermalNodeDataArray::SharedPtr msg) {
          for (const auto & n : msg->nodes) {
            temperature[n.name] = n.temperature;
            power[n.name] = n.internal_power;
          }
        }));
    subs.push_back(node->create_subscription<ThermalLinkFlowsArray>(
        "/thermal/links/flux", 10, [this](ThermalLinkFlowsArray::SharedPtr msg) {
          for (const auto & l : msg->links) {
            conductance[l.node_a] = l.conductance;
          }
        }));
    subs.push_back(node->create_subscription<SubsystemHeartbeat>(
        "/ssos/thermal/heartbeat", 10, [this](SubsystemHeartbeat::SharedPtr msg) {
          ++heartbeats;
          healthy = msg->healthy;
          status = msg->status_message;
        }));
    subs.push_back(node->create_subscription<FaultEvent>(
        "/ssos/fault_event", 10, [this](FaultEvent::SharedPtr msg) {
          overheat_events += msg->fault_type == "thermal_node_overheat";
        }));
  }

  void drill(const std::string & type, const std::string & params, double duration_s,
             const std::string & target = "thermal")
  {
    FaultEvent ev;
    ev.subsystem_name = target;
    ev.fault_type = type;
    ev.description = ssos_sim::fault_description("Injected fault", type, params, duration_s);
    fault_pub->publish(ev);
  }

  void heat(const std::string & node_name, double power_w)
  {
    space_station_interfaces::msg::HeatLoadArray msg;
    msg.source = "test";
    msg.loads.resize(1);
    msg.loads[0].node_name = node_name;
    msg.loads[0].power = power_w;
    heat_pub->publish(msg);
  }

  void clock(double seconds)
  {
    rosgraph_msgs::msg::Clock msg;
    msg.clock = rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
    clock_pub->publish(msg);
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<FaultEvent>::SharedPtr fault_pub;
  rclcpp::Publisher<space_station_interfaces::msg::HeatLoadArray>::SharedPtr heat_pub;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  std::map<std::string, double> temperature, power, conductance;
  int heartbeats = 0;
  bool healthy = true;
  std::string status;
  int overheat_events = 0;
};

std::shared_ptr<ThermalNetworkNode> make_node(std::vector<rclcpp::Parameter> params)
{
  params.emplace_back("thermal_update_dt", 0.05);
  rclcpp::NodeOptions options;
  options.parameter_overrides(params);
  return std::make_shared<ThermalNetworkNode>(options);
}

}  // namespace

class ThermalNetworkNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides({source_config()});
    node_ = std::make_shared<ThermalNetworkNode>(options);
  }

  std::shared_ptr<ThermalNetworkNode> node_;
};

TEST_F(ThermalNetworkNodeTest, ConfigureActivateDeactivateCleanup)
{
  using lifecycle_msgs::msg::State;
  EXPECT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  spin_node_for(node_->get_node_base_interface(), 200ms);
  EXPECT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(ThermalNetworkNodeAutostartTest, ConfiguresAndActivatesWithoutExternalCall)
{
  using lifecycle_msgs::msg::State;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("autostart", true), rclcpp::Parameter("autostart_delay_ms", 50),
      source_config()});
  auto node = std::make_shared<ThermalNetworkNode>(options);

  spin_node_for(node->get_node_base_interface(), 500ms);
  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
}

class ThermalNetworkProbeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    probe_ = std::make_unique<Probe>();
    exec_.add_node(probe_->node);
  }

  void start(const std::shared_ptr<ThermalNetworkNode> & node)
  {
    node_ = node;
    exec_.add_node(node_->get_node_base_interface());
    ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    // The probe's own fault subscription plus thermal_network's.
    ASSERT_TRUE(spin_until(exec_, [this] {
        return probe_->fault_pub->get_subscription_count() >= 2 &&
        probe_->heat_pub->get_subscription_count() >= 1 && probe_->heartbeats > 0;
      }, 5s));
  }

  bool near(const std::map<std::string, double> & values, const std::string & key,
            double expected)
  {
    const auto it = values.find(key);
    return it != values.end() && std::abs(it->second - expected) < 1e-6;
  }

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::unique_ptr<Probe> probe_;
  std::shared_ptr<ThermalNetworkNode> node_;
};

TEST_F(ThermalNetworkProbeTest, DrillsApplyAndRecover)
{
  start(make_node({source_config("station_thermal_nodes.yaml")}));
  auto & p = *probe_;
  ASSERT_TRUE(spin_until(exec_, [&] {return near(p.temperature, "itcs_sink", 17.0);}, 2s));

  // Another subsystem's drill is not ours.
  p.drill("itcs_loop_degraded", "{}", 1.0, "eps");
  spin_until(exec_, [] {return false;}, 300ms);
  EXPECT_TRUE(near(p.temperature, "itcs_sink", 17.0));

  p.drill("itcs_loop_degraded", "{}", 1.0);
  EXPECT_TRUE(spin_until(exec_, [&] {return near(p.temperature, "itcs_sink", 26.0);}, 2s));
  EXPECT_TRUE(spin_until(exec_, [&] {return near(p.temperature, "itcs_sink", 17.0);}, 3s));

  // No duration: active until clear_faults.
  p.drill("coldplate_conductance_loss", "{\"factor\": 0.1}", 0.0);
  EXPECT_TRUE(spin_until(exec_, [&] {return near(p.conductance, "itcs_coldplate", 100.0);}, 2s));
  spin_until(exec_, [] {return false;}, 300ms);
  EXPECT_TRUE(near(p.conductance, "itcs_coldplate", 100.0));
  p.drill(ssos_sim::kClearFaults, "{}", 0.0);
  EXPECT_TRUE(spin_until(exec_, [&] {return near(p.conductance, "itcs_coldplate", 1000.0);}, 2s));

  // Unknown drills are ignored.
  p.drill("itcs_pump_overspeed", "{\"factor\": 0.1}", 0.0);
  spin_until(exec_, [] {return false;}, 300ms);
  EXPECT_TRUE(near(p.conductance, "itcs_coldplate", 1000.0));
  EXPECT_TRUE(near(p.temperature, "itcs_sink", 17.0));
}

TEST_F(ThermalNetworkProbeTest, DrillSurvivesDeactivateAndActivate)
{
  using lifecycle_msgs::msg::State;
  start(make_node({source_config("station_thermal_nodes.yaml")}));
  auto & p = *probe_;

  p.drill("itcs_loop_degraded", "{}", 0.0);
  ASSERT_TRUE(spin_until(exec_, [&] {return near(p.temperature, "itcs_sink", 26.0);}, 2s));
  ASSERT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  p.temperature.clear();
  ASSERT_TRUE(spin_until(exec_, [&] {return p.temperature.count("itcs_sink") > 0;}, 2s));
  EXPECT_TRUE(near(p.temperature, "itcs_sink", 26.0));

  // A drill that arrives while inactive replaces the old one on activation.
  ASSERT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  p.drill("coldplate_conductance_loss", "{}", 0.0);
  spin_until(exec_, [] {return false;}, 300ms);
  ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  p.temperature.clear();
  p.conductance.clear();
  ASSERT_TRUE(spin_until(exec_, [&] {return p.temperature.count("itcs_sink") > 0;}, 2s));
  EXPECT_TRUE(near(p.temperature, "itcs_sink", 17.0));
  EXPECT_TRUE(near(p.conductance, "itcs_coldplate", 350.0));

  p.drill(ssos_sim::kClearFaults, "{}", 0.0);
  EXPECT_TRUE(spin_until(exec_, [&] {return near(p.conductance, "itcs_coldplate", 1000.0);}, 2s));
}

TEST_F(ThermalNetworkProbeTest, OverLimitRaisesOneFaultAndClears)
{
  const auto path = (std::filesystem::temp_directory_path() /
    "ssos_thermal_test_limit.yaml").string();
  {
    std::ofstream out(path);
    out <<
      "- {node_name: sink, parent_link: \"\", heat_capacity: 1.0e6, internal_power: 0.0,\n"
      "   conductance: 0.0, fixed_temperature_c: 17.0}\n"
      "- {node_name: plate, parent_link: sink, heat_capacity: 10.0, internal_power: 0.0,\n"
      "   conductance: 10.0, initial_temperature_c: 17.0, max_temperature_c: 30.0}\n";
  }
  start(make_node({rclcpp::Parameter("thermal_config_file", path)}));
  auto & p = *probe_;
  EXPECT_TRUE(p.healthy);

  // 500 W settles the plate at 67 degC with a 1 s time constant.
  p.heat("plate", 500.0);
  ASSERT_TRUE(spin_until(exec_, [&] {return !p.healthy;}, 3s));
  EXPECT_NE(p.status.find("plate"), std::string::npos);
  spin_until(exec_, [] {return false;}, 500ms);
  EXPECT_FALSE(p.healthy);

  p.heat("plate", 0.0);
  EXPECT_TRUE(spin_until(exec_, [&] {return p.healthy;}, 5s));
  EXPECT_LT(p.temperature["plate"], 29.0);
  EXPECT_EQ(p.overheat_events, 1);
  std::filesystem::remove(path);
}

TEST_F(ThermalNetworkProbeTest, PausedSimTimeDoesNoPhysicsButKeepsHeartbeats)
{
  auto node = make_node({source_config(), rclcpp::Parameter("use_sim_time", true)});
  // Hold the clock at 100 s before the first step so nothing ever advances.
  ASSERT_TRUE(spin_until(exec_, [this] {
      probe_->clock(100.0);
      return probe_->clock_pub->get_subscription_count() >= 1;
    }, 5s));
  ASSERT_TRUE(spin_until(exec_, [&] {return node->now().seconds() == 100.0;}, 2s));
  start(node);
  auto & p = *probe_;

  const int beats = p.heartbeats;
  spin_until(exec_, [] {return false;}, 1s);
  EXPECT_GE(p.heartbeats - beats, 5);
  EXPECT_DOUBLE_EQ(p.temperature["base_link"], 20.0);

  p.clock(110.0);
  EXPECT_TRUE(spin_until(exec_, [&] {return p.temperature["base_link"] > 20.0;}, 2s));
}

TEST_F(ThermalNetworkProbeTest, RetriesRegistrationUntilAccepted)
{
  // The first request is never answered, the second is rejected, the third
  // is accepted.
  int requests = 0;
  rclcpp::Service<RegisterSubsystem>::SharedPtr service;
  service = probe_->node->create_service<RegisterSubsystem>(
    "/ssos/register_subsystem",
    [&](std::shared_ptr<rmw_request_id_t> header, RegisterSubsystem::Request::SharedPtr req) {
      EXPECT_EQ(req->subsystem_name, "thermal");
      if (++requests == 1) {
        return;
      }
      RegisterSubsystem::Response res;
      res.accepted = requests == 3;
      service->send_response(*header, res);
    });
  start(make_node({source_config()}));
  EXPECT_TRUE(spin_until(exec_, [&] {return requests == 3;}, 8s));
  spin_until(exec_, [] {return false;}, 500ms);
  EXPECT_EQ(requests, 3);
}
