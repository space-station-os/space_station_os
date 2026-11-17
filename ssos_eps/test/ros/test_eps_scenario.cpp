#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "space_station_interfaces/msg/system_state.hpp"
#include "ssos_core/system_manager.hpp"
#include "ssos_sim/simulation_controller.hpp"

#include "ssos_eps/nodes/eps_node.hpp"

// End-to-end: system_manager + simulation_controller running a scenario file
// + eps_node, all in one process. The scenario injects an EPS fault through
// the simulation path; system_manager must go NOMINAL -> DEGRADED -> RECOVERY
// -> NOMINAL once the fault's duration runs out.

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using space_station_interfaces::msg::FaultEvent;
using space_station_interfaces::msg::SystemState;

class EpsScenarioTest : public ::testing::Test
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

  void start(const std::string & scenario)
  {
    manager_ = std::make_shared<ssos_core::SystemManager>(
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("recovery_hold_s", 1.0),
        rclcpp::Parameter("eval_period_s", 0.2)}));
    sim_ = std::make_shared<ssos_sim::SimulationController>(
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("scenario_file",
        std::string(SSOS_EPS_TEST_SCENARIOS) + "/" + scenario)}));
    eps_ = std::make_shared<ssos_eps::nodes::EpsNode>(
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("use_sim_time", true),
        rclcpp::Parameter("step_rate_hz", 20.0)}));
    probe_ = std::make_shared<rclcpp::Node>("scenario_probe");
    state_sub_ = probe_->create_subscription<SystemState>(
      "/ssos/system_state", 10, [this](SystemState::SharedPtr m) {states_.push_back(m->state);});
    fault_sub_ = probe_->create_subscription<FaultEvent>(
      "/ssos/fault_event", 10, [this](FaultEvent::SharedPtr m) {faults_.push_back(*m);});

    executor_.add_node(manager_->get_node_base_interface());
    executor_.add_node(sim_->get_node_base_interface());
    executor_.add_node(eps_->get_node_base_interface());
    executor_.add_node(probe_);

    ASSERT_EQ(manager_->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(manager_->activate().id(), State::PRIMARY_STATE_ACTIVE);
    ASSERT_EQ(eps_->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(eps_->activate().id(), State::PRIMARY_STATE_ACTIVE);
    ASSERT_EQ(sim_->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(sim_->activate().id(), State::PRIMARY_STATE_ACTIVE);
  }

  void TearDown() override
  {
    if (manager_) {
      executor_.remove_node(probe_);
      executor_.remove_node(eps_->get_node_base_interface());
      executor_.remove_node(sim_->get_node_base_interface());
      executor_.remove_node(manager_->get_node_base_interface());
    }
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end && rclcpp::ok()) {
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  // True if the states appear in this order (not necessarily adjacent).
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

  bool saw_fault(const std::string & type, const std::string & description_prefix) const
  {
    return std::any_of(faults_.begin(), faults_.end(), [&](const FaultEvent & f) {
        return f.subsystem_name == "eps" && f.fault_type == type &&
               f.description.rfind(description_prefix, 0) == 0;
      });
  }

  std::shared_ptr<ssos_core::SystemManager> manager_;
  std::shared_ptr<ssos_sim::SimulationController> sim_;
  std::shared_ptr<ssos_eps::nodes::EpsNode> eps_;
  rclcpp::Node::SharedPtr probe_;
  rclcpp::Subscription<SystemState>::SharedPtr state_sub_;
  rclcpp::Subscription<FaultEvent>::SharedPtr fault_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<uint8_t> states_;
  std::vector<FaultEvent> faults_;
};

TEST_F(EpsScenarioTest, BcduFaultDegradesAndRecovers)
{
  start("e2e_bcdu_fault.yaml");

  // Sim runs 20x: the fault starts at T+30 s (~1.5 s wall) for 20 s (~1 s).
  spin_for(1300ms);
  EXPECT_EQ(manager_->get_current_state(), SystemState::NOMINAL);

  spin_for(700ms);
  EXPECT_EQ(manager_->get_current_state(), SystemState::DEGRADED);
  EXPECT_TRUE(eps_->last_state().bcdu_fault);
  EXPECT_EQ(eps_->last_state().channels[5].mode, ssos_eps::bcdu::BcduMode::FAULT);
  // The injection from the sim, and EPS's own detection of it.
  EXPECT_TRUE(saw_fault("bcdu_fault", "Scheduled fault: bcdu_channel_5"));
  EXPECT_TRUE(saw_fault("bcdu_fault", "A BCDU has isolated"));

  // Fault expires at ~2.5 s wall, then a 1 s healthy hold.
  spin_for(2500ms);
  EXPECT_FALSE(eps_->last_state().bcdu_fault);
  EXPECT_EQ(manager_->get_current_state(), SystemState::NOMINAL);
  EXPECT_TRUE(saw_in_order({SystemState::NOMINAL, SystemState::DEGRADED,
      SystemState::RECOVERY, SystemState::NOMINAL}));
}

TEST_F(EpsScenarioTest, LoadSpikeShedsLoadsAndRecovers)
{
  start("e2e_load_spike.yaml");

  spin_for(2000ms);
  EXPECT_TRUE(eps_->last_state().load_shed);
  EXPECT_GT(eps_->last_state().load_demand_w, 59000.0);
  EXPECT_EQ(manager_->get_current_state(), SystemState::DEGRADED);

  spin_for(2500ms);
  EXPECT_FALSE(eps_->last_state().load_shed);
  EXPECT_NEAR(eps_->last_state().load_demand_w, 30000.0, 1.0);
  EXPECT_EQ(manager_->get_current_state(), SystemState::NOMINAL);
  EXPECT_TRUE(saw_in_order({SystemState::DEGRADED, SystemState::RECOVERY,
      SystemState::NOMINAL}));
}
