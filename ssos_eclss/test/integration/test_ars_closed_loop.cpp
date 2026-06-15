#include <gtest/gtest.h>

#include "ssos_eclss/ars/four_bed_system.hpp"
#include "ssos_eclss/cabin/cabin_atmosphere.hpp"
#include "ssos_eclss/cabin/crew_metabolic_model.hpp"
#include "ssos_eclss/common/units.hpp"

using namespace ssos_eclss;

// Closed loop: crew add CO2 to the cabin, the ARS scrubs it out. Verify the ARS
// removes CO2 and the cabin ppCO2 stays bounded (does not run away).
TEST(ArsClosedLoop, RemovesCrewCO2)
{
  cabin::CabinParams cp{};
  cp.volume_m3 = 90.0;
  cp.temperature_k = units::celsius_to_kelvin(22.0);
  cabin::CabinAtmosphere atm(cp);
  atm.initialize_nominal(3000.0, 0.40);  // start somewhat elevated

  cabin::CrewMetabolicModel crew(3, cabin::default_crew_profile());

  // Coarse, fast ARS configuration.
  ars::ArsParameters p = ars::default_ars_parameters();
  p.desiccant_bed.n_cells = 8;
  p.adsorbent_bed.n_cells = 8;
  ars::FourBedSystem fbs(p);

  const double dt = 2.0;
  const double co2_ppm_start = atm.co2_ppm();
  double total_removed_kg = 0.0;

  for (int i = 0; i < 600; ++i) {  // 1200 s
    ars::CabinConditions cc{};
    cc.co2_partial_pressure_pa = atm.partial_pressure_pa(cabin::Gas::CO2);
    cc.h2o_partial_pressure_pa = atm.partial_pressure_pa(cabin::Gas::H2O);
    cc.temperature_k = atm.temperature_k();
    cc.total_pressure_pa = atm.total_pressure_pa();

    ars::ArsResult r = fbs.step(dt, cc);
    total_removed_kg += r.co2_removal_rate_kg_s * dt;

    const cabin::MetabolicLoads m = crew.loads(1.0);
    cabin::GasFlows f{};
    f.co2 = m.flows.co2 - r.co2_removal_rate_kg_s / units::M_CO2;
    f.o2 = m.flows.o2;
    f.h2o = m.flows.h2o;
    atm.apply_flows(dt, f);
  }

  EXPECT_GT(total_removed_kg, 0.0);
  // CO2 should not have run away (ARS keeps it bounded / falling).
  EXPECT_LT(atm.co2_ppm(), co2_ppm_start + 2000.0);
}
