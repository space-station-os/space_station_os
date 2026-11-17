#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ssos_sim/simulation_controller.hpp"
#include "space_station_interfaces/msg/world_state.hpp"
#include "space_station_interfaces/msg/fault_event.hpp"
#include "space_station_interfaces/srv/inject_fault.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std::chrono_literals;
using WorldState = space_station_interfaces::msg::WorldState;
using FaultEvent = space_station_interfaces::msg::FaultEvent;
using InjectFault = space_station_interfaces::srv::InjectFault;
using Clock = rosgraph_msgs::msg::Clock;

class SimControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    sim_ = std::make_shared<ssos_sim::SimulationController>();

    sim_->configure();
    sim_->activate();

    client_ = std::make_shared<rclcpp::Node>("test_client");
    inject_client_ = client_->create_client<InjectFault>("/sim/inject_fault");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(sim_->get_node_base_interface());
    executor_->add_node(client_);
  }

  void TearDown() override
  {
    sim_.reset();
    client_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some(10ms);
    }
  }

  std::shared_ptr<ssos_sim::SimulationController> sim_;
  std::shared_ptr<rclcpp::Node> client_;
  rclcpp::Client<InjectFault>::SharedPtr inject_client_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(SimControllerTest, StartsAtTimeZero)
{
  EXPECT_DOUBLE_EQ(sim_->get_sim_time(), 0.0);
}

TEST_F(SimControllerTest, SimTimeAdvances)
{
  // Let it step a few times
  spin_for(500ms);
  EXPECT_GT(sim_->get_sim_time(), 0.0);
}

TEST_F(SimControllerTest, WorldStatePublished)
{
  bool received = false;
  auto sub = client_->create_subscription<WorldState>(
    "/sim/world_state", 10,
    [&received](const WorldState::SharedPtr) { received = true; });

  spin_for(500ms);
  EXPECT_TRUE(received);
}

TEST_F(SimControllerTest, ClockPublished)
{
  bool received = false;
  auto sub = client_->create_subscription<Clock>(
    "/clock", 10,
    [&received](const Clock::SharedPtr) { received = true; });

  spin_for(500ms);
  EXPECT_TRUE(received);
}

TEST_F(SimControllerTest, WorldStateHasValidDefaults)
{
  WorldState ws;
  bool received = false;
  auto sub = client_->create_subscription<WorldState>(
    "/sim/world_state", 10,
    [&ws, &received](const WorldState::SharedPtr msg) {
      ws = *msg;
      received = true;
    });

  spin_for(500ms);
  ASSERT_TRUE(received);
  EXPECT_NEAR(ws.altitude_km, 408.0, 0.1);
  EXPECT_NEAR(ws.orbital_velocity_mps, 7660.0, 1.0);
  EXPECT_NEAR(ws.atmospheric_o2_pct, 20.9, 0.1);
  EXPECT_NEAR(ws.cabin_pressure_kpa, 101.3, 0.1);
}

TEST_F(SimControllerTest, InjectFaultServiceWorks)
{
  ASSERT_TRUE(inject_client_->wait_for_service(2s));

  auto req = std::make_shared<InjectFault::Request>();
  req->target_subsystem = "eclss";
  req->fault_type = "sensor_stuck_at";
  req->parameters_json = "{\"stuck_value\": 0.0}";
  req->duration_s = 0.0;

  auto future = inject_client_->async_send_request(req);
  spin_for(500ms);

  ASSERT_EQ(future.wait_for(0s), std::future_status::ready);
  auto res = future.get();
  EXPECT_TRUE(res->success);
}

TEST_F(SimControllerTest, FaultInjectionPublishesFaultEvent)
{
  ASSERT_TRUE(inject_client_->wait_for_service(2s));

  bool fault_received = false;
  auto fault_sub = client_->create_subscription<FaultEvent>(
    "/ssos/fault_event", 10,
    [&fault_received](const FaultEvent::SharedPtr) { fault_received = true; });

  spin_for(200ms);

  auto req = std::make_shared<InjectFault::Request>();
  req->target_subsystem = "eclss";
  req->fault_type = "sensor_stuck_at";
  req->parameters_json = "{\"stuck_value\": 0.0}";
  req->duration_s = 0.0;

  inject_client_->async_send_request(req);
  spin_for(500ms);

  EXPECT_TRUE(fault_received);
}

TEST_F(SimControllerTest, InjectedFaultDescriptionCarriesParamsAndDuration)
{
  ASSERT_TRUE(inject_client_->wait_for_service(2s));
  std::string description;
  auto fault_sub = client_->create_subscription<FaultEvent>(
    "/ssos/fault_event", 10,
    [&description](const FaultEvent::SharedPtr m) {description = m->description;});
  spin_for(200ms);

  auto req = std::make_shared<InjectFault::Request>();
  req->target_subsystem = "eps";
  req->fault_type = "bcdu_fault";
  req->parameters_json = "{\"channel\": 2}";
  req->duration_s = 45.0;
  inject_client_->async_send_request(req);
  spin_for(500ms);

  EXPECT_EQ(description,
            "Injected fault: bcdu_fault (params: {\"channel\": 2}, duration: 45 s)");
}

class SimScenarioTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    client_ = std::make_shared<rclcpp::Node>("test_client");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(client_);
  }

  void TearDown() override
  {
    if (sim_) {
      executor_->remove_node(sim_->get_node_base_interface());
    }
    sim_.reset();
    client_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  void make_sim(const std::string & scenario_file)
  {
    sim_ = std::make_shared<ssos_sim::SimulationController>(
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("scenario_file", scenario_file)}));
    executor_->add_node(sim_->get_node_base_interface());
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some(10ms);
    }
  }

  std::shared_ptr<ssos_sim::SimulationController> sim_;
  std::shared_ptr<rclcpp::Node> client_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(SimScenarioTest, LoadsScenarioAndFiresFaultsOnSchedule)
{
  make_sim(std::string(SSOS_SIM_TEST_SCENARIOS) + "/test_schedule.yaml");
  std::vector<FaultEvent> faults;
  auto fault_sub = client_->create_subscription<FaultEvent>(
    "/ssos/fault_event", 10, [&faults](const FaultEvent::SharedPtr m) {faults.push_back(*m);});

  ASSERT_EQ(sim_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(sim_->scenario_name(), "test_schedule");
  ASSERT_EQ(sim_->fault_schedule().size(), 2u);
  EXPECT_DOUBLE_EQ(sim_->get_parameter("time_scale").as_double(), 50.0);
  EXPECT_NEAR(sim_->get_world_model().get_state().atmospheric_co2_ppm, 2600.0, 1e-9);
  // orbit_phase 0.7 starts the run in eclipse.
  EXPECT_TRUE(sim_->get_world_model().get_state().in_eclipse);

  ASSERT_EQ(sim_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  // 20 Hz x 50 = 2.5 sim-s per step: the 10 s fault fires within ~0.2 s wall
  // and the 40 s fault within ~0.8 s.
  spin_for(400ms);
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].subsystem_name, "eps");
  EXPECT_EQ(faults[0].fault_type, "bcdu_fault");
  EXPECT_EQ(faults[0].description,
            "Scheduled fault: early_fault (params: {\"channel\": 3, \"note\": \"quoted\", "
            "\"clear\": false}, duration: 0 s)");
  ASSERT_EQ(faults[0].affected_interfaces.size(), 1u);
  EXPECT_EQ(faults[0].affected_interfaces[0], "/ssos/eps/bcdu/status");

  spin_for(800ms);
  ASSERT_EQ(faults.size(), 2u);
  EXPECT_EQ(faults[1].fault_type, "load_spike");
  EXPECT_GE(sim_->get_sim_time(), 40.0);
  EXPECT_TRUE(sim_->fault_schedule()[1].fired);
}

TEST_F(SimScenarioTest, ShippedScenarioResolvesByName)
{
  make_sim("eps_bcdu_fault_recovery");
  ASSERT_EQ(sim_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(sim_->scenario_name(), "eps_bcdu_fault_recovery");
  ASSERT_EQ(sim_->fault_schedule().size(), 1u);
  EXPECT_EQ(sim_->fault_schedule()[0].fault_type, "bcdu_fault");
}

TEST_F(SimScenarioTest, BadScenarioFailsConfigure)
{
  make_sim(std::string(SSOS_SIM_TEST_SCENARIOS) + "/bad_trigger.yaml");
  EXPECT_EQ(sim_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}