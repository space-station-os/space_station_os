#include <gtest/gtest.h>

#include "ssos_eclss/cabin/cabin_atmosphere.hpp"
#include "ssos_eclss/cabin/crew_metabolic_model.hpp"
#include "ssos_eclss/cabin/leak_model.hpp"
#include "ssos_eclss/common/units.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::cabin;

// Mass conservation is mandatory for a life-support model. This test runs the
// cabin under crew CO2 production, ARS-style CO2 removal and leakage, and
// verifies the CO2 budget closes exactly:
//   CO2_in = CO2_accumulated + CO2_removed + CO2_leaked.
TEST(EclssMassBalance, CO2BudgetCloses)
{
  CabinParams cp{};
  cp.volume_m3 = 100.0;
  cp.temperature_k = units::celsius_to_kelvin(22.0);
  CabinAtmosphere atm(cp);

  CrewMetabolicModel crew(4, default_crew_profile());

  LeakParams lp{};
  lp.nominal_area_m2 = 1.0e-7;
  lp.discharge_coeff = 0.62;
  LeakModel leak(lp);

  // Constant ARS CO2 removal sink [mol/s] (design point).
  const double ars_removal_mol_s = units::kg_per_day_to_kg_per_s(4.3) / units::M_CO2;

  const double co2_initial = atm.moles(Gas::CO2);
  double co2_in = 0.0, co2_removed = 0.0, co2_leaked = 0.0;

  const double dt = 1.0;
  for (int i = 0; i < 3600; ++i) {  // 1 hour
    const MetabolicLoads m = crew.loads(1.0);
    const GasFlows lk = leak.leak_flows(atm);

    GasFlows total{};
    total.o2 = m.flows.o2 + lk.o2;
    total.co2 = m.flows.co2 - ars_removal_mol_s + lk.co2;
    total.n2 = lk.n2;
    total.h2o = m.flows.h2o + lk.h2o;

    co2_in += m.flows.co2 * dt;
    co2_removed += ars_removal_mol_s * dt;
    co2_leaked += -lk.co2 * dt;  // lk.co2 is negative (leaving)

    atm.apply_flows(dt, total);
  }

  const double co2_accumulated = atm.moles(Gas::CO2) - co2_initial;
  const double residual = co2_in - (co2_accumulated + co2_removed + co2_leaked);
  // Should close to within floating-point round-off.
  EXPECT_NEAR(residual, 0.0, 1.0e-6 * (co2_in + 1.0));
}

TEST(EclssMassBalance, TotalGasMassConservedWithoutSources)
{
  CabinParams cp{};
  cp.volume_m3 = 100.0;
  cp.temperature_k = units::celsius_to_kelvin(22.0);
  CabinAtmosphere atm(cp);

  const double m0 = atm.total_gas_mass_kg();
  // No flows applied -> mass unchanged.
  atm.apply_flows(100.0, GasFlows{});
  EXPECT_NEAR(atm.total_gas_mass_kg(), m0, 1.0e-12);
}

TEST(EclssMassBalance, LeakReducesPressure)
{
  CabinParams cp{};
  cp.volume_m3 = 100.0;
  cp.temperature_k = units::celsius_to_kelvin(22.0);
  CabinAtmosphere atm(cp);

  LeakParams lp{};
  lp.nominal_area_m2 = 1.0e-4;  // large leak
  lp.discharge_coeff = 0.62;
  LeakModel leak(lp);

  const double p0 = atm.total_pressure_pa();
  for (int i = 0; i < 600; ++i) {
    atm.apply_flows(1.0, leak.leak_flows(atm));
  }
  EXPECT_LT(atm.total_pressure_pa(), p0);
}
