#include <gtest/gtest.h>

#include "ssos_eclss/cabin/atmosphere_control.hpp"
#include "ssos_eclss/cabin/cabin_atmosphere.hpp"
#include "ssos_eclss/cabin/leak_model.hpp"
#include "ssos_eclss/common/units.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::cabin;

namespace
{
CabinAtmosphere make_cabin()
{
  CabinParams cp{};
  cp.volume_m3 = 388.0;
  cp.temperature_k = units::celsius_to_kelvin(22.0);
  CabinAtmosphere atm(cp);
  atm.initialize_nominal(2600.0, 0.45);
  return atm;
}

// Run the cabin for `seconds` with a constant crew latent load and a leak.
void run(CabinAtmosphere & atm, AtmosphereControl & ctl, const LeakModel & leak,
         double latent_kg_day, double seconds, double dt = 60.0)
{
  const double latent_mol_s = units::kg_per_day_to_kg_per_s(latent_kg_day) / units::M_H2O;
  for (double t = 0.0; t < seconds; t += dt) {
    const ControlResult c = ctl.step(dt, atm);
    const GasFlows lk = leak.leak_flows(atm);
    GasFlows f{};
    f.o2 = lk.o2 + c.flows.o2;
    f.co2 = lk.co2;
    f.n2 = lk.n2 + c.flows.n2;
    f.h2o = latent_mol_s + lk.h2o + c.flows.h2o;
    atm.apply_flows(dt, f);
  }
}
}  // namespace

TEST(AtmosphereControl, ChxHoldsHumidityAgainstTheCrewLatentLoad)
{
  CabinAtmosphere atm = make_cabin();
  AtmosphereControl ctl;
  LeakModel leak(LeakParams{0.0, 0.62});
  run(atm, ctl, leak, 8.8, 3 * 86400.0);
  EXPECT_NEAR(atm.relative_humidity(), 0.45, 0.02);
}

TEST(AtmosphereControl, ChxFailureLetsHumidityRise)
{
  CabinAtmosphere atm = make_cabin();
  AtmosphereControl ctl;
  ctl.set_chx_failed(true);
  LeakModel leak(LeakParams{0.0, 0.62});
  run(atm, ctl, leak, 8.8, 8 * 3600.0);
  EXPECT_GT(atm.relative_humidity(), 0.75);
}

TEST(AtmosphereControl, NitrogenMakeupHoldsPressureAgainstALeak)
{
  CabinAtmosphere atm = make_cabin();
  AtmosphereControl ctl;
  LeakModel leak(LeakParams{2.0e-8, 0.62});
  const double n2_start = ctl.n2_tank_kg();
  run(atm, ctl, leak, 0.0, 30 * 86400.0, 300.0);   // about 7 kg of air lost
  const double p_kpa = units::pa_to_kpa(atm.total_pressure_pa());
  EXPECT_GT(p_kpa, 100.5);
  EXPECT_LT(p_kpa, 101.5);
  EXPECT_LT(ctl.n2_tank_kg(), n2_start);
}

TEST(AtmosphereControl, OxygenMakeupOnlyBelowTheFloor)
{
  CabinAtmosphere atm = make_cabin();
  AtmosphereControl ctl;
  EXPECT_DOUBLE_EQ(ctl.step(60.0, atm).o2_makeup_kg_s, 0.0);
  atm.add_moles(Gas::O2, -0.2 * atm.moles(Gas::O2));   // ppO2 about 16.8 kPa
  EXPECT_GT(ctl.step(60.0, atm).o2_makeup_kg_s, 0.0);
}
