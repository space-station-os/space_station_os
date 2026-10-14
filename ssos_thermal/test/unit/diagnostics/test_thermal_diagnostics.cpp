#include <gtest/gtest.h>

#include "ssos_thermal/nodes/thermal_diagnostics.hpp"

using ssos_thermal::nodes::ThermalDiagnostics;

TEST(ThermalDiagnosticsTest, NoFaultWhileHealthy)
{
  ThermalDiagnostics diag;
  for (int i = 0; i < 5; ++i) {
    EXPECT_FALSE(diag.should_raise_fault(true));
  }
}

TEST(ThermalDiagnosticsTest, RaisesExactlyOnceWhileFaultStaysActive)
{
  ThermalDiagnostics diag;
  EXPECT_TRUE(diag.should_raise_fault(false));
  for (int i = 0; i < 5; ++i) {
    EXPECT_FALSE(diag.should_raise_fault(false));
  }
}

TEST(ThermalDiagnosticsTest, RaisesAgainAfterRecoveryAndNewFault)
{
  ThermalDiagnostics diag;
  EXPECT_TRUE(diag.should_raise_fault(false));
  EXPECT_FALSE(diag.should_raise_fault(false));
  EXPECT_FALSE(diag.should_raise_fault(true));
  EXPECT_TRUE(diag.should_raise_fault(false));
}
