#include <gtest/gtest.h>

#include <map>
#include <string>

#include "ssos_eps/nodes/eps_fault_injection.hpp"

using ssos_eps::nodes::EpsFaultInjection;

TEST(FaultInjection, RecognisesSimulationDescriptions)
{
  std::string json;
  double duration = -1.0;
  EXPECT_TRUE(EpsFaultInjection::parse_description(
      "Injected fault: bcdu_fault (params: {\"channel\": 2}, duration: 45 s)", json, duration));
  EXPECT_EQ(json, "{\"channel\": 2}");
  EXPECT_DOUBLE_EQ(duration, 45.0);

  EXPECT_TRUE(EpsFaultInjection::parse_description(
      "Scheduled fault: spike (params: {}, duration: 0 s)", json, duration));
  EXPECT_EQ(json, "{}");
  EXPECT_DOUBLE_EQ(duration, 0.0);

  // EPS's own detected faults and anything else are not injections.
  EXPECT_FALSE(EpsFaultInjection::parse_description(
      "A BCDU has isolated its batteries", json, duration));
}

TEST(FaultInjection, ParsesFlatJson)
{
  const auto p = EpsFaultInjection::parse_params(
    "{\"channel\": 3, \"note\": \"text, with comma\", \"clear\": true, \"x\": -1.5e1}");
  EXPECT_DOUBLE_EQ(p.at("channel"), 3.0);
  EXPECT_DOUBLE_EQ(p.at("clear"), 1.0);
  EXPECT_DOUBLE_EQ(p.at("x"), -15.0);
  EXPECT_EQ(p.count("note"), 0u);
  EXPECT_TRUE(EpsFaultInjection::parse_params("{}").empty());
}

TEST(FaultInjection, AppliesAndExpires)
{
  EpsFaultInjection inj;
  std::string msg;
  ASSERT_TRUE(inj.apply("bcdu_fault", {{"channel", 4.0}}, 30.0, 100.0, msg));
  ASSERT_TRUE(inj.apply("load_spike", {}, 0.0, 100.0, msg));
  EXPECT_EQ(inj.indices("bcdu_fault"), (std::set<int>{4}));
  EXPECT_DOUBLE_EQ(inj.demand_scale(), 2.0);

  EXPECT_FALSE(inj.expire(129.0));
  EXPECT_TRUE(inj.expire(130.0));
  EXPECT_TRUE(inj.indices("bcdu_fault").empty());
  // No duration: stays until cleared.
  EXPECT_DOUBLE_EQ(inj.demand_scale(), 2.0);
}

TEST(FaultInjection, PerUnitFaultsAreKeyedByIndex)
{
  EpsFaultInjection inj;
  std::string msg;
  inj.apply("mbsu_channel_loss", {{"channel", 0.0}}, 0.0, 0.0, msg);
  inj.apply("mbsu_channel_loss", {{"channel", 1.0}}, 0.0, 0.0, msg);
  inj.apply("mbsu_channel_loss", {{"channel", 1.0}}, 10.0, 0.0, msg);
  EXPECT_EQ(inj.active().size(), 2u);
  EXPECT_EQ(inj.indices("mbsu_channel_loss"), (std::set<int>{0, 1}));
  // Re-injecting the same unit replaced it with the timed version.
  inj.expire(10.0);
  EXPECT_EQ(inj.indices("mbsu_channel_loss"), (std::set<int>{0}));
}

TEST(FaultInjection, ClearOneTypeOrEverything)
{
  EpsFaultInjection inj;
  std::string msg;
  inj.apply("ddcu_fault", {{"ddcu", 2.0}}, 0.0, 0.0, msg);
  inj.apply("ddcu_fault", {{"ddcu", 5.0}}, 0.0, 0.0, msg);
  inj.apply("solar_array_degraded", {{"factor", 0.4}}, 0.0, 0.0, msg);
  EXPECT_DOUBLE_EQ(inj.array_factor(), 0.4);

  inj.apply("ddcu_fault", {{"ddcu", 2.0}, {"clear", 1.0}}, 0.0, 0.0, msg);
  EXPECT_EQ(inj.indices("ddcu_fault"), (std::set<int>{5}));

  const auto rev = inj.revision();
  ASSERT_TRUE(inj.apply("clear_faults", {}, 0.0, 0.0, msg));
  EXPECT_TRUE(inj.active().empty());
  EXPECT_GT(inj.revision(), rev);
  EXPECT_DOUBLE_EQ(inj.array_factor(), 1.0);
}

TEST(FaultInjection, DefaultsAndOneShotSoc)
{
  EpsFaultInjection inj;
  std::string msg;
  inj.apply("battery_degraded", {}, 0.0, 0.0, msg);
  inj.apply("solar_array_degraded", {}, 0.0, 0.0, msg);
  EXPECT_DOUBLE_EQ(inj.health_factor(), 0.5);
  EXPECT_DOUBLE_EQ(inj.array_factor(), 0.3);

  ASSERT_TRUE(inj.apply("battery_low_soc", {{"soc", 0.12}}, 0.0, 0.0, msg));
  auto soc = inj.take_pending_soc();
  ASSERT_TRUE(soc.has_value());
  EXPECT_DOUBLE_EQ(*soc, 0.12);
  EXPECT_FALSE(inj.take_pending_soc().has_value());
  EXPECT_EQ(inj.active().size(), 2u);
}

TEST(FaultInjection, RejectsUnknownTypes)
{
  EpsFaultInjection inj;
  std::string msg;
  EXPECT_FALSE(inj.apply("sensor_stuck_at", {}, 0.0, 0.0, msg));
  EXPECT_NE(msg.find("unsupported"), std::string::npos);
  EXPECT_TRUE(inj.active().empty());
}
