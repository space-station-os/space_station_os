#include <gtest/gtest.h>

#include "ssos_eclss/ars/adsorption_isotherm.hpp"
#include "ssos_eclss/ars/bed_model.hpp"
#include "ssos_eclss/common/gas_properties.hpp"
#include "ssos_eclss/common/units.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::ars;

namespace
{
BedModel make_adsorbent_bed()
{
  BedGeometry g = default_adsorbent_geometry();
  g.n_cells = 30;  // keep tests fast
  TothIsotherm co2(default_co2_on_13x());
  TothIsotherm h2o(default_h2o_on_13x());
  return BedModel(g, default_adsorbent_thermal(), co2, h2o,
                  default_adsorbent_ldf(), /*is_desiccant=*/false);
}

BedInlet make_inlet()
{
  BedInlet in{};
  in.temperature_k = 295.0;
  in.pressure_pa = units::torr_to_pa(760.0);
  in.velocity_superficial = 0.4;
  in.c_co2 = gas::concentration_from_pp(units::torr_to_pa(3.0), 295.0);
  in.c_h2o = 0.0;  // dry (downstream of desiccant)
  return in;
}
}  // namespace

TEST(BedModel, FreshBedAdsorbsCO2)
{
  BedModel bed = make_adsorbent_bed();
  bed.reset(295.0);
  BedInlet in = make_inlet();

  // Run a short transient.
  BedOutputs out{};
  for (int i = 0; i < 20; ++i) {
    out = bed.step(1.0, in, BedMode::ADSORBING, 0.0);
  }
  // A fresh bed should remove most CO2: outlet < inlet.
  EXPECT_LT(out.outlet_c_co2, in.c_co2);
  // And it should be accumulating CO2.
  EXPECT_GT(bed.total_co2_loading_mol(), 0.0);
}

TEST(BedModel, CaptureRateNonNegative)
{
  BedModel bed = make_adsorbent_bed();
  bed.reset(295.0);
  BedInlet in = make_inlet();
  BedOutputs out = bed.step(1.0, in, BedMode::ADSORBING, 0.0);
  EXPECT_GE(out.co2_capture_rate, 0.0);
}

TEST(BedModel, HeaterRaisesBedTemperature)
{
  BedModel bed = make_adsorbent_bed();
  bed.reset(295.0);
  const double t0 = bed.mean_solid_temperature();

  BedInlet in = make_inlet();
  in.velocity_superficial = 0.0;
  in.pressure_pa = units::torr_to_pa(2.0);
  for (int i = 0; i < 300; ++i) {
    bed.step(2.0, in, BedMode::DESORBING, 700.0);
  }
  EXPECT_GT(bed.mean_solid_temperature(), t0 + 50.0);
}

TEST(BedModel, DesorptionRemovesLoading)
{
  BedModel bed = make_adsorbent_bed();
  bed.equilibrate(units::torr_to_pa(3.0), 0.0, 295.0);
  const double loaded = bed.total_co2_loading_mol();
  EXPECT_GT(loaded, 0.0);

  BedInlet in = make_inlet();
  in.velocity_superficial = 0.0;
  in.pressure_pa = units::torr_to_pa(2.0);
  for (int i = 0; i < 400; ++i) {
    bed.step(2.0, in, BedMode::DESORBING, 700.0);
  }
  // Regeneration should drive loading well below the adsorbed level.
  EXPECT_LT(bed.total_co2_loading_mol(), 0.5 * loaded);
}

TEST(BedModel, PressureDropPositiveWhenFlowing)
{
  BedModel bed = make_adsorbent_bed();
  BedInlet in = make_inlet();
  BedOutputs out = bed.step(1.0, in, BedMode::ADSORBING, 0.0);
  EXPECT_GT(out.pressure_drop_pa, 0.0);
}

TEST(BedModel, MassNotCreatedFromNothing)
{
  // With zero inlet CO2 and a clean bed, no CO2 may appear.
  BedModel bed = make_adsorbent_bed();
  bed.reset(295.0);
  BedInlet in = make_inlet();
  in.c_co2 = 0.0;
  for (int i = 0; i < 10; ++i) {
    bed.step(1.0, in, BedMode::ADSORBING, 0.0);
  }
  EXPECT_NEAR(bed.total_co2_loading_mol(), 0.0, 1.0e-9);
}
