#include <gtest/gtest.h>

#include "ssos_eclss/cabin/cabin_atmosphere.hpp"
#include "ssos_eclss/common/units.hpp"

using namespace ssos_eclss;
using namespace ssos_eclss::cabin;

namespace
{
CabinAtmosphere make_cabin()
{
  CabinParams p{};
  p.volume_m3 = 100.0;
  p.temperature_k = units::celsius_to_kelvin(22.0);
  return CabinAtmosphere(p);
}
}  // namespace

TEST(CabinAtmosphere, NominalCompositionReasonable)
{
  CabinAtmosphere atm = make_cabin();
  EXPECT_NEAR(atm.total_pressure_pa(), units::STD_PRESSURE_PA, 200.0);
  EXPECT_NEAR(atm.o2_fraction(), 0.21, 0.02);
  EXPECT_GT(atm.co2_ppm(), 0.0);
  EXPECT_LT(atm.co2_ppm(), 5000.0);
}

TEST(CabinAtmosphere, CO2ProductionRaisesPP)
{
  CabinAtmosphere atm = make_cabin();
  const double pp0 = atm.partial_pressure_pa(Gas::CO2);
  GasFlows f{};
  f.co2 = 1.0e-3;  // mol/s into cabin
  atm.apply_flows(60.0, f);
  EXPECT_GT(atm.partial_pressure_pa(Gas::CO2), pp0);
}

TEST(CabinAtmosphere, CO2RemovalLowersPP)
{
  CabinAtmosphere atm = make_cabin();
  const double pp0 = atm.partial_pressure_pa(Gas::CO2);
  GasFlows f{};
  f.co2 = -1.0e-4;  // mol/s out of cabin
  atm.apply_flows(60.0, f);
  EXPECT_LT(atm.partial_pressure_pa(Gas::CO2), pp0);
}

TEST(CabinAtmosphere, MolesNeverNegative)
{
  CabinAtmosphere atm = make_cabin();
  GasFlows f{};
  f.co2 = -1.0e9;  // absurd sink
  atm.apply_flows(1.0, f);
  EXPECT_GE(atm.moles(Gas::CO2), 0.0);
}

TEST(CabinAtmosphere, MoleConservationOnAddRemove)
{
  CabinAtmosphere atm = make_cabin();
  const double n0 = atm.moles(Gas::O2);
  atm.add_moles(Gas::O2, 5.0);
  EXPECT_NEAR(atm.moles(Gas::O2), n0 + 5.0, 1.0e-9);
  atm.add_moles(Gas::O2, -2.0);
  EXPECT_NEAR(atm.moles(Gas::O2), n0 + 3.0, 1.0e-9);
}

TEST(CabinAtmosphere, HumidityAndDewPointConsistent)
{
  CabinAtmosphere atm = make_cabin();
  const double rh = atm.relative_humidity();
  EXPECT_GT(rh, 0.0);
  EXPECT_LT(rh, 1.0);
  // Dew point must be below cabin temperature for sub-saturated air.
  EXPECT_LT(atm.dew_point_k(), atm.temperature_k());
}

TEST(CabinAtmosphere, PartialPressuresSumToTotal)
{
  CabinAtmosphere atm = make_cabin();
  const double sum = atm.partial_pressure_pa(Gas::O2) + atm.partial_pressure_pa(Gas::CO2) +
                     atm.partial_pressure_pa(Gas::N2) + atm.partial_pressure_pa(Gas::H2O);
  EXPECT_NEAR(sum, atm.total_pressure_pa(), 1.0e-6 * atm.total_pressure_pa());
}
