#include <gtest/gtest.h>

#include "ssos_eclss/common/units.hpp"
#include "ssos_eclss/wrs/catalytic_reactor_model.hpp"
#include "ssos_eclss/wrs/distillation_model.hpp"
#include "ssos_eclss/wrs/multifiltration_model.hpp"
#include "ssos_eclss/wrs/water_recovery_system.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::wrs;

TEST(Distillation, RecoversConfiguredFraction)
{
  DistillationModel d(default_distillation_params());
  const double feed = 5.0 / 86400.0;  // 5 kg/day
  const DistillationResult r = d.process(feed);
  EXPECT_NEAR(r.distillate_kg_s, feed * 0.87, 1.0e-12);
  EXPECT_NEAR(r.distillate_kg_s + r.brine_kg_s, feed, 1.0e-12);
  EXPECT_GT(r.energy_w, 0.0);
}

TEST(Multifiltration, ReducesConductivity)
{
  MultifiltrationModel mf(default_multifiltration_params());
  mf.reset();
  const MultifiltrationResult r = mf.process(1.0, 1.0e-4, 2000.0);
  EXPECT_LT(r.product_conductivity_us, 2000.0);
}

TEST(Multifiltration, EventualBreakthrough)
{
  MultifiltrationParams p = default_multifiltration_params();
  p.bed_capacity_kg = 1.0e-3;  // small bed
  MultifiltrationModel mf(p);
  mf.reset();
  MultifiltrationResult r{};
  // Push a lot of contaminated water through.
  for (int i = 0; i < 100000 && !r.broken_through; ++i) {
    r = mf.process(1.0, 1.0, 5000.0);
  }
  EXPECT_TRUE(r.broken_through);
}

TEST(Catalytic, ConversionRisesWithTemperature)
{
  CatalyticReactorModel c(default_catalytic_params());
  EXPECT_LT(c.conversion(340.0), c.conversion(420.0));
  EXPECT_LE(c.conversion(420.0), default_catalytic_params().max_conversion + 1.0e-9);
  EXPECT_NEAR(c.conversion(300.0), 0.0, 1.0e-9);  // below activation
}

TEST(WaterRecovery, ProducesPotableWater)
{
  WaterRecoverySystem wrs;
  wrs.reset();
  const double urine = 5.0 / 86400.0;
  const double condensate = 3.0 / 86400.0;
  const WrsResult r = wrs.step(1.0, urine, condensate, 1.0e-8);
  EXPECT_GT(r.potable_water_kg_s, 0.0);
  EXPECT_TRUE(r.potable_in_spec);
  EXPECT_GT(r.voc_conversion, 0.5);
}

TEST(WaterRecovery, RecoveryFractionReasonable)
{
  WaterRecoverySystem wrs;
  wrs.reset();
  const double urine = 5.0 / 86400.0;
  const double condensate = 3.0 / 86400.0;
  const WrsResult r = wrs.step(1.0, urine, condensate, 0.0);
  // Combined recovery should be high (condensate ~100%, urine ~87%).
  EXPECT_GT(r.overall_recovery, 0.85);
  EXPECT_LE(r.overall_recovery, 1.0);
}
