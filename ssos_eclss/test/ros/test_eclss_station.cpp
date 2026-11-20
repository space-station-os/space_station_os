#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/msg/subsystem_heartbeat.hpp"
#include "ssos_sim/fault_format.hpp"

#include "ssos_eclss/nodes/ars_node.hpp"
#include "ssos_eclss/nodes/cabin_node.hpp"
#include "ssos_eclss/nodes/crew_node.hpp"
#include "ssos_eclss/nodes/ogs_node.hpp"
#include "ssos_eclss/nodes/sabatier_node.hpp"
#include "ssos_eclss/nodes/wrs_node.hpp"

// The ECLSS loop in one process on a simulation clock the test drives:
// crew, cabin, ARS, OGS, WRS and Sabatier, without EPS (units run as fully
// powered). Checks the settled on-orbit start, every ECLSS drill, and that a
// paused clock stops the physics.

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticArray;
using lifecycle_msgs::msg::State;
using rosgraph_msgs::msg::Clock;
using space_station_interfaces::msg::FaultEvent;
using space_station_interfaces::msg::SubsystemHeartbeat;

namespace
{
const std::vector<std::string> kUnits = {"crew", "cabin", "ars", "ogs", "wrs", "sabatier"};
}

class EclssStationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {rclcpp::shutdown();}

  template<typename NodeT>
  void make()
  {
    auto node = std::make_shared<NodeT>(rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("use_sim_time", true),
        rclcpp::Parameter("step_rate_hz", 20.0)}));
    executor_.add_node(node->get_node_base_interface());
    nodes_.push_back(node);
  }

  void SetUp() override
  {
    probe_ = std::make_shared<rclcpp::Node>("eclss_probe");
    executor_.add_node(probe_);
    clock_pub_ = probe_->create_publisher<Clock>("/clock", 10);
    fault_pub_ = probe_->create_publisher<FaultEvent>("/ssos/fault_event", 10);
    fault_sub_ = probe_->create_subscription<FaultEvent>(
      "/ssos/fault_event", 50, [this](FaultEvent::SharedPtr m) {
        if (!ssos_sim::parse_injected_fault(m->fault_type, m->description)) {
          detected_[m->subsystem_name].push_back(m->fault_type);
        }
      });
    for (const auto & unit : kUnits) {
      hb_subs_.push_back(probe_->create_subscription<SubsystemHeartbeat>(
          "/ssos/" + unit + "/heartbeat", 10,
          [this, unit](SubsystemHeartbeat::SharedPtr m) {healthy_[unit] = m->healthy;}));
      diag_subs_.push_back(probe_->create_subscription<DiagnosticArray>(
          "/ssos/" + unit + "/diagnostics", 10, [this](DiagnosticArray::SharedPtr m) {
            for (const auto & st : m->status) {
              for (const auto & kv : st.values) {
                diag_[st.name][kv.key] = kv.value;
              }
            }
          }));
    }
    publish_clock();

    make<ssos_eclss::nodes::CrewNode>();
    make<ssos_eclss::nodes::CabinNode>();
    make<ssos_eclss::nodes::ArsNode>();
    make<ssos_eclss::nodes::OgsNode>();
    make<ssos_eclss::nodes::WrsNode>();
    make<ssos_eclss::nodes::SabatierNode>();
    for (auto & n : nodes_) {
      ASSERT_EQ(n->configure().id(), State::PRIMARY_STATE_INACTIVE) << n->get_name();
      ASSERT_EQ(n->activate().id(), State::PRIMARY_STATE_ACTIVE) << n->get_name();
    }
  }

  void TearDown() override
  {
    for (auto & n : nodes_) {
      executor_.remove_node(n->get_node_base_interface());
    }
    executor_.remove_node(probe_);
  }

  void publish_clock()
  {
    Clock c;
    c.clock.sec = static_cast<int32_t>(sim_s_);
    c.clock.nanosec = static_cast<uint32_t>((sim_s_ - c.clock.sec) * 1e9);
    clock_pub_->publish(c);
  }

  // Advance sim time in small increments, letting every node step in between.
  void advance(double sim_seconds, double per_tick_s = 30.0)
  {
    for (double t = 0.0; t < sim_seconds; t += per_tick_s) {
      sim_s_ += per_tick_s;
      publish_clock();
      spin(60ms);
    }
  }

  bool advance_until(const std::function<bool()> & done, double max_sim_s)
  {
    for (double t = 0.0; t < max_sim_s; t += 60.0) {
      if (done()) {
        return true;
      }
      advance(60.0);
    }
    return done();
  }

  void spin(std::chrono::milliseconds d)
  {
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some();
      std::this_thread::sleep_for(2ms);
    }
  }

  void inject(const std::string & target, const std::string & type, const std::string & json)
  {
    FaultEvent m;
    m.subsystem_name = target;
    m.fault_type = type;
    m.description = ssos_sim::fault_description("Injected fault", type, json, 0.0);
    fault_pub_->publish(m);
    spin(100ms);
  }

  double diag(const std::string & unit, const std::string & key)
  {
    return std::stod(diag_.at(unit).at(key));
  }

  bool all_healthy()
  {
    for (const auto & u : kUnits) {
      if (!healthy_.count(u) || !healthy_[u]) {
        return false;
      }
    }
    return true;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> nodes_;
  rclcpp::Node::SharedPtr probe_;
  rclcpp::Publisher<Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<FaultEvent>::SharedPtr fault_pub_;
  rclcpp::Subscription<FaultEvent>::SharedPtr fault_sub_;
  std::vector<rclcpp::Subscription<SubsystemHeartbeat>::SharedPtr> hb_subs_;
  std::vector<rclcpp::Subscription<DiagnosticArray>::SharedPtr> diag_subs_;
  std::map<std::string, bool> healthy_;
  std::map<std::string, std::vector<std::string>> detected_;
  std::map<std::string, std::map<std::string, std::string>> diag_;
  double sim_s_{100.0};
};

TEST_F(EclssStationTest, StartsSettledAndStaysNominal)
{
  advance(600.0);
  ASSERT_TRUE(all_healthy());
  advance(3 * 3600.0, 60.0);
  EXPECT_TRUE(all_healthy());
  for (const auto & [unit, faults] : detected_) {
    for (const auto & f : faults) {
      ADD_FAILURE() << unit << " raised " << f;
    }
  }

  EXPECT_NEAR(diag("cabin", "relative_humidity"), 0.45, 0.04);
  EXPECT_NEAR(diag("cabin", "o2_fraction_mca"), 0.207, 0.005);
  EXPECT_NEAR(diag("cabin", "total_pressure_kpa"), 101.3, 0.4);
  EXPECT_LT(diag("cabin", "co2_ppm"), 3500.0);
  EXPECT_GT(diag("cabin", "condensate_kg_day"), 5.0);
  // What the GUI's crew and water cards show.
  EXPECT_EQ(diag_.at("crew").count("activity"), 1u);
  EXPECT_NEAR(diag("crew", "crew_size"), 4.0, 1e-9);
  EXPECT_GT(diag("wrs", "potable_available_kg"), 350.0);
  EXPECT_GT(diag("wrs", "days_of_supply"), 20.0);
  EXPECT_EQ(diag_.at("wrs").count("wastewater_kg"), 1u);
  EXPECT_EQ(diag_.at("wrs").count("upa_processing"), 1u);
}

TEST_F(EclssStationTest, EveryDrillDegradesItsUnitOnceAndRecovers)
{
  struct Drill
  {
    std::string target, type, json;
    double max_s;   // sim time allowed to show the fault, and to recover
  };
  const std::vector<Drill> drills = {
    {"crew", "crew_metabolic_surge", "{\"scale\": 2.0}", 600},
    {"cabin", "cabin_leak", "{\"area_m2\": 3e-6}", 600},
    {"cabin", "o2_sensor_stuck", "{\"value_pct\": 23.0}", 600},
    {"cabin", "chx_failure", "{}", 12 * 3600},
    // ARS health follows capture efficiency averaged over about a half-cycle.
    {"ars", "ars_scrubber_degraded", "{\"capture_efficiency\": 0.2}", 4 * 3600},
    {"ars", "ars_blower_failure", "{}", 600},
    {"ars", "ars_heater_failure", "{}", 12 * 3600},
    {"ogs", "ogs_generation_failure", "{}", 600},
    {"wrs", "upa_assembly_failure", "{}", 600},
    {"wrs", "wpa_multifiltration_breakthrough", "{}", 600},
    {"sabatier", "sabatier_reactor_failure", "{}", 600},
  };
  advance(600.0);
  ASSERT_TRUE(all_healthy());
  for (const auto & d : drills) {
    SCOPED_TRACE(d.type);
    detected_.clear();
    inject(d.target, d.type, d.json);
    EXPECT_TRUE(advance_until([&] {return !healthy_[d.target];}, d.max_s));
    advance(120.0);
    EXPECT_EQ(detected_[d.target].size(), 1u);
    inject(d.target, "clear_faults", "{}");
    EXPECT_TRUE(advance_until([&] {return all_healthy();}, d.max_s));
  }
}

TEST_F(EclssStationTest, PausedClockFreezesThePhysics)
{
  advance(600.0);
  spin(500ms);   // let the last tick's telemetry arrive
  const double co2 = diag("cabin", "co2_ppm");
  const double potable = diag("wrs", "potable_available_kg");
  int beats = 0;
  auto sub = probe_->create_subscription<SubsystemHeartbeat>(
    "/ssos/cabin/heartbeat", 10, [&beats](SubsystemHeartbeat::SharedPtr) {++beats;});
  spin(1500ms);   // clock not advanced
  EXPECT_GT(beats, 5);
  EXPECT_DOUBLE_EQ(diag("cabin", "co2_ppm"), co2);
  EXPECT_DOUBLE_EQ(diag("wrs", "potable_available_kg"), potable);
}
