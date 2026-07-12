#include <gtest/gtest.h>

#include "ssos_eclss/cabin/crew_metabolic_model.hpp"
#include "ssos_eclss/common/units.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::cabin;

TEST(CrewMetabolic, CO2ProducedO2Consumed)
{
  CrewMetabolicModel crew(4, default_crew_profile());
  const MetabolicLoads l = crew.loads(1.0);
  EXPECT_GT(l.flows.co2, 0.0);   // CO2 produced
  EXPECT_LT(l.flows.o2, 0.0);    // O2 consumed
  EXPECT_GT(l.flows.h2o, 0.0);   // water produced
  EXPECT_GT(l.heat_w, 0.0);
}

TEST(CrewMetabolic, ScalesWithCrewSize)
{
  CrewMetabolicModel one(1, default_crew_profile());
  CrewMetabolicModel four(4, default_crew_profile());
  EXPECT_NEAR(four.loads().flows.co2, 4.0 * one.loads().flows.co2, 1.0e-12);
}

TEST(CrewMetabolic, ActivityFactorScales)
{
  CrewMetabolicModel crew(3, default_crew_profile());
  EXPECT_NEAR(crew.loads(2.0).flows.co2, 2.0 * crew.loads(1.0).flows.co2, 1.0e-12);
}

TEST(CrewMetabolic, MagnitudeMatchesDailyFigures)
{
  CrewMetabolicModel crew(1, default_crew_profile());
  const MetabolicLoads l = crew.loads(1.0);
  // CO2 ~1 kg/day per crew member.
  const double co2_kg_day = l.flows.co2 * units::M_CO2 * units::SECONDS_PER_DAY;
  EXPECT_NEAR(co2_kg_day, 1.04, 0.05);
  const double o2_kg_day = -l.flows.o2 * units::M_O2 * units::SECONDS_PER_DAY;
  EXPECT_NEAR(o2_kg_day, 0.84, 0.05);
}
