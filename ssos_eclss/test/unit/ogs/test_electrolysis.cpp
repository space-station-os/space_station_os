#include <gtest/gtest.h>

#include "ssos_eclss/common/units.hpp"
#include "ssos_eclss/ogs/electrolysis_cell_model.hpp"
#include "ssos_eclss/ogs/oxygen_generator_system.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::ogs;

TEST(Electrolysis, ReversibleVoltageNear1p23)
{
  ElectrolysisCellModel cell(default_cell_params());
  EXPECT_NEAR(cell.reversible_voltage(298.15, units::STD_PRESSURE_PA), 1.229, 0.05);
}

TEST(Electrolysis, VoltageExceedsReversibleUnderLoad)
{
  ElectrolysisCellModel cell(default_cell_params());
  const CellState s = cell.solve(20.0, 330.0, units::STD_PRESSURE_PA);
  EXPECT_GT(s.voltage, s.reversible_voltage);
  EXPECT_GT(s.activation_overpotential, 0.0);
  EXPECT_GT(s.ohmic_overpotential, 0.0);
}

TEST(Electrolysis, VoltageMonotonicInCurrent)
{
  ElectrolysisCellModel cell(default_cell_params());
  const CellState lo = cell.solve(5.0, 330.0, units::STD_PRESSURE_PA);
  const CellState hi = cell.solve(40.0, 330.0, units::STD_PRESSURE_PA);
  EXPECT_GT(hi.voltage, lo.voltage);
}

TEST(Electrolysis, FaradayStoichiometry)
{
  ElectrolysisCellModel cell(default_cell_params());
  const CellState s = cell.solve(20.0, 330.0, units::STD_PRESSURE_PA);
  // H2 should be twice O2 (2 H2 + O2 from 2 H2O).
  EXPECT_NEAR(s.h2_production_mol_s, 2.0 * s.o2_production_mol_s, 1.0e-12);
  // Per cell, O2 = Faradaic_efficiency * I/(4F).
  const double fe = default_cell_params().faradaic_efficiency;
  EXPECT_NEAR(s.o2_production_mol_s, fe * 20.0 / (4.0 * units::FARADAY), 1.0e-12);
}

TEST(Electrolysis, SystemProducesOxygen)
{
  OxygenGeneratorSystem ogs;
  ogs.reset(330.0);
  OgsResult r{};
  for (int i = 0; i < 100; ++i) {
    r = ogs.step_nominal(1.0);
  }
  EXPECT_GT(r.o2_production_kg_day, 0.0);
  // OGA-class production a few kg/day at the nominal 27 A.
  EXPECT_GT(r.o2_production_kg_day, 2.0);
  EXPECT_LT(r.o2_production_kg_day, 12.0);
  // 2 H2O consumed per O2.
  EXPECT_NEAR(r.water_consumed_mol_s, 2.0 * r.o2_production_mol_s, 1.0e-12);
}

TEST(Electrolysis, MaxRateMatchesAOGA)
{
  // ICES-2023-311: 28-cell OGA delivers 9.25 kg O2/day at 46.9 A.
  OgsParameters p = default_ogs_parameters();
  OxygenGeneratorSystem ogs(p);
  ogs.reset(330.0);
  OgsResult r{};
  for (int i = 0; i < 50; ++i) {
    r = ogs.step(1.0, 46.9, 1.0e9);
  }
  EXPECT_NEAR(r.o2_production_kg_day, 9.25, 0.3);
}

TEST(Electrolysis, CellVoltageNearAOGAOperatingPoint)
{
  // Nominal cell voltage ~1.7 V at the 46.9 A operating point (AOGA endurance
  // test). Evaluate a single cell at the stack temperature and loop pressure.
  ElectrolysisCellModel cell(default_cell_params());
  const CellState s = cell.solve(46.9, 330.0, 165474.0);
  EXPECT_NEAR(s.voltage, 1.7, 0.15);
}

TEST(Electrolysis, FeedwaterLimiting)
{
  OxygenGeneratorSystem ogs;
  ogs.reset(330.0);
  // Provide almost no feed water.
  const OgsResult r = ogs.step(1.0, 27.0, 1.0e-9);
  EXPECT_TRUE(r.feedwater_limited);
  EXPECT_LT(r.o2_production_mol_s, 1.0e-6);
}

TEST(Electrolysis, StackHeatsUnderLoad)
{
  OxygenGeneratorSystem ogs;
  ogs.reset(300.0);
  OgsResult r{};
  for (int i = 0; i < 500; ++i) {
    r = ogs.step_nominal(1.0);
  }
  // Waste heat above thermoneutral raises stack temperature above coolant.
  EXPECT_GT(r.stack_temperature_k, units::celsius_to_kelvin(18.0));
}
