#include <gtest/gtest.h>

#include "ssos_eclss/ars/four_bed_system.hpp"
#include "ssos_eclss/common/gas_properties.hpp"
#include "ssos_eclss/common/units.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::ars;

namespace
{
CabinConditions nominal_cabin()
{
  CabinConditions c{};
  c.co2_partial_pressure_pa = units::torr_to_pa(2.0);
  c.h2o_partial_pressure_pa = gas::water_pp_from_rh(0.40, units::celsius_to_kelvin(22.0));
  c.temperature_k = units::celsius_to_kelvin(22.0);
  c.total_pressure_pa = units::torr_to_pa(760.0);
  return c;
}
}  // namespace

TEST(FourBedSystem, DesignRemovalMatchesPaper)
{
  FourBedSystem ars;
  const double removal = ars.design_co2_removal_kg_day();
  // ICES-2021-313: 4.16 - 4.76 kg/day at 2 torr, 26 SCFM.
  EXPECT_GE(removal, 4.16);
  EXPECT_LE(removal, 4.76);
}

TEST(FourBedSystem, FreshSystemRemovesCO2)
{
  FourBedSystem ars;
  ars.reset(units::celsius_to_kelvin(22.0));
  CabinConditions cabin = nominal_cabin();

  ArsResult r{};
  for (int i = 0; i < 60; ++i) {
    r = ars.step(1.0, cabin);
  }
  // A fresh adsorbing bed scrubs CO2: removal positive and within the band.
  EXPECT_GT(r.co2_removal_rate_kg_day, 0.0);
  EXPECT_LE(r.co2_removal_rate_kg_day, 5.2);
  // Return air is drier/cleaner than the cabin.
  EXPECT_LT(r.scrubbed_co2_pp_pa, cabin.co2_partial_pressure_pa);
}

TEST(FourBedSystem, PressureDropInRange)
{
  FourBedSystem ars;
  CabinConditions cabin = nominal_cabin();
  ArsResult r = ars.step(1.0, cabin);
  const double dp_in_h2o = units::pa_to_in_h2o(r.system_pressure_drop_pa);
  // System dP order of magnitude check (paper ~37-40 in-H2O total path;
  // bed-only component is a fraction of that).
  EXPECT_GT(dp_in_h2o, 0.0);
  EXPECT_LT(dp_in_h2o, 200.0);
}

TEST(FourBedSystem, RegeneratingBedHeatsUp)
{
  // Use a coarse, short-cycle configuration so the desorb heaters act quickly
  // (keeps the transient cheap; the physics is identical to the 50-cell case).
  ArsParameters p = default_ars_parameters();
  p.desiccant_bed.n_cells = 10;
  p.adsorbent_bed.n_cells = 10;
  p.cycle.air_save_s = 60.0;
  p.cycle.adsorb_s = 300.0;
  p.cycle.vacuum_s = 60.0;
  FourBedSystem ars(p);
  ars.reset(units::celsius_to_kelvin(22.0));
  CabinConditions cabin = nominal_cabin();

  const double t0 = ars.adsorbent_bed(1).mean_solid_temperature();
  for (int i = 0; i < 150; ++i) {
    ars.step(2.0, cabin);  // 300 s -> through air-save into desorb
  }
  EXPECT_GT(ars.adsorbent_bed(1).mean_solid_temperature(), t0 + 20.0);
}

TEST(FourBedSystem, BlowerFlowNearDesign)
{
  FourBedSystem ars;
  CabinConditions cabin = nominal_cabin();
  ArsResult r = ars.step(1.0, cabin);
  EXPECT_NEAR(r.blower_flow_scfm, 26.0, 3.0);
}

TEST(FourBedSystem, PrecoolerExitCool)
{
  FourBedSystem ars;
  CabinConditions cabin = nominal_cabin();
  ArsResult r = ars.step(1.0, cabin);
  // Precooler exit below cabin and above LTL inlet (7 C).
  EXPECT_LT(r.precooler_exit_temp_k, cabin.temperature_k);
  EXPECT_GT(r.precooler_exit_temp_k, units::celsius_to_kelvin(6.0));
}
