#include <gtest/gtest.h>

#include <vector>

#include "ssos_eps/loads/load_registry.hpp"

using namespace ssos_eps::loads;

TEST(LoadRegistry, UpsertAddsAndReplaces)
{
  LoadRegistry r;
  r.upsert({"a", 1000.0, 1, 0});
  r.upsert({"b", 500.0, 2, 1});
  EXPECT_EQ(r.loads().size(), 2u);
  r.upsert({"a", 1500.0, 0, 1});
  ASSERT_EQ(r.loads().size(), 2u);
  ASSERT_NE(r.find("a"), nullptr);
  EXPECT_DOUBLE_EQ(r.find("a")->power_w, 1500.0);
  EXPECT_EQ(r.find("a")->ddcu, 1);
  EXPECT_EQ(r.find("missing"), nullptr);
}

TEST(LoadRegistry, RemoveUnknownReturnsFalse)
{
  LoadRegistry r;
  r.upsert({"a", 1000.0, 1, 0});
  EXPECT_FALSE(r.remove("b"));
  EXPECT_TRUE(r.remove("a"));
  EXPECT_TRUE(r.loads().empty());
}

TEST(LoadRegistry, TotalsHonourDemandScale)
{
  LoadRegistry r;
  r.upsert({"a", 1000.0, 1, 0});
  r.upsert({"b", 500.0, 2, 0});
  r.upsert({"c", 2000.0, 0, 1});
  EXPECT_DOUBLE_EQ(r.total_requested_w(), 3500.0);
  EXPECT_DOUBLE_EQ(r.requested_on_ddcu(0), 1500.0);
  EXPECT_DOUBLE_EQ(r.requested_on_ddcu(0, "a"), 500.0);
  r.set_demand_scale(2.0);
  EXPECT_DOUBLE_EQ(r.total_requested_w(), 7000.0);
  r.set_demand_scale(-1.0);
  EXPECT_DOUBLE_EQ(r.demand_scale(), 0.0);
}

TEST(LoadRegistry, ShedsLeastCriticalThenLargestFirst)
{
  LoadRegistry r;
  r.upsert({"critical", 3000.0, 0, 0});
  r.upsert({"payload_small", 1000.0, 3, 0});
  r.upsert({"crew", 2000.0, 2, 0});
  r.upsert({"payload_big", 4000.0, 3, 0});
  const auto order = r.shed_order({0, 1, 2, 3});
  ASSERT_EQ(order.size(), 4u);
  EXPECT_EQ(r.loads()[order[0]].id, "payload_big");
  EXPECT_EQ(r.loads()[order[1]].id, "payload_small");
  EXPECT_EQ(r.loads()[order[2]].id, "crew");
  EXPECT_EQ(r.loads()[order[3]].id, "critical");
}
