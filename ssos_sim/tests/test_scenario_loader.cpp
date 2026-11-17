#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "ssos_sim/scenario_loader.hpp"

using ssos_sim::fault_description;
using ssos_sim::load_scenario_file;

namespace
{
std::string test_file(const std::string & name)
{
  return std::string(SSOS_SIM_TEST_SCENARIOS) + "/" + name;
}
}  // namespace

TEST(ScenarioLoader, ParsesSettingsAndInitialConditions)
{
  const auto s = load_scenario_file(test_file("test_schedule.yaml"));
  EXPECT_EQ(s.name, "test_schedule");
  ASSERT_TRUE(s.duration_s.has_value());
  EXPECT_DOUBLE_EQ(*s.duration_s, 120.0);
  EXPECT_DOUBLE_EQ(*s.sim_rate_hz, 20.0);
  EXPECT_DOUBLE_EQ(*s.time_scale, 50.0);
  EXPECT_DOUBLE_EQ(*s.orbit_phase, 0.7);
  EXPECT_DOUBLE_EQ(s.initial_conditions.at("atmospheric_co2_ppm"), 2600.0);
  EXPECT_FALSE(s.in_eclipse.has_value());
}

TEST(ScenarioLoader, SortsFaultsAndConvertsParamsToJson)
{
  const auto s = load_scenario_file(test_file("test_schedule.yaml"));
  ASSERT_EQ(s.faults.size(), 2u);
  EXPECT_EQ(s.faults[0].name, "early_fault");
  EXPECT_EQ(s.faults[0].fault_type, "bcdu_fault");
  EXPECT_EQ(s.faults[0].target_subsystem, "eps");
  EXPECT_EQ(s.faults[0].target_interface, "/ssos/eps/bcdu/status");
  EXPECT_DOUBLE_EQ(s.faults[0].trigger_time_s, 10.0);
  EXPECT_DOUBLE_EQ(s.faults[0].duration_s, 0.0);
  EXPECT_EQ(s.faults[0].parameters_json,
            "{\"channel\": 3, \"note\": \"quoted\", \"clear\": false}");
  EXPECT_EQ(s.faults[1].name, "late_fault");
  EXPECT_EQ(s.faults[1].parameters_json, "{\"demand_scale\": 2.0}");
  EXPECT_DOUBLE_EQ(s.faults[1].duration_s, 30.0);
}

TEST(ScenarioLoader, RejectsMissingFileAndNonTimeTrigger)
{
  EXPECT_THROW(load_scenario_file(test_file("does_not_exist.yaml")), std::runtime_error);
  EXPECT_THROW(load_scenario_file(test_file("bad_trigger.yaml")), std::runtime_error);
}

TEST(ScenarioLoader, ShippedScenariosLoad)
{
  for (const char * name : {"nominal_scene.yaml", "eclss_o2_sensor.yaml",
      "eps_eclipse_transition.yaml", "eps_bcdu_fault_recovery.yaml",
      "eps_load_spike.yaml", "eps_channel_loss.yaml", "eps_low_battery_recovery.yaml"})
  {
    const std::string path = std::string(SSOS_SIM_SCENARIOS) + "/" + name;
    EXPECT_NO_THROW(load_scenario_file(path)) << path;
  }
}

TEST(ScenarioLoader, FaultDescriptionFormat)
{
  EXPECT_EQ(fault_description("Injected fault", "bcdu_fault", "{\"channel\": 2}", 60.0),
            "Injected fault: bcdu_fault (params: {\"channel\": 2}, duration: 60 s)");
  EXPECT_EQ(fault_description("Scheduled fault", "x", "", 0.0),
            "Scheduled fault: x (params: {}, duration: 0 s)");
}
