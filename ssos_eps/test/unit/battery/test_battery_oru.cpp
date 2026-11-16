#include <gtest/gtest.h>

#include <cmath>

#include "ssos_eps/battery/battery_oru_model.hpp"

using namespace ssos_eps::battery;

TEST(BatteryOru, OpenCircuitVoltageFollowsTable)
{
  const BatteryOruParams p = default_battery_oru_params();
  BatteryOruModel oru(p);
  EXPECT_NEAR(oru.open_circuit_voltage(0.0), p.n_series_cells * 3.00, 1.0e-9);
  EXPECT_NEAR(oru.open_circuit_voltage(1.0), p.n_series_cells * 4.10, 1.0e-9);
  EXPECT_NEAR(oru.open_circuit_voltage(0.55), p.n_series_cells * 0.5 * (3.72 + 3.80), 1.0e-9);
  double last = 0.0;
  for (int i = 0; i <= 100; ++i) {
    const double v = oru.open_circuit_voltage(i / 100.0);
    EXPECT_GE(v, last);
    last = v;
  }
}

TEST(BatteryOru, DischargeFollowsCoulombCounting)
{
  const BatteryOruParams p = default_battery_oru_params();
  BatteryOruModel oru(p);
  const double soc0 = oru.state().soc;
  const double delivered = oru.step(60.0, 2000.0);
  const BatteryOruState & s = oru.state();
  EXPECT_NEAR(delivered, 2000.0, 1.0e-6);
  EXPECT_GT(s.current_a, 0.0);
  EXPECT_EQ(s.mode, BatteryMode::DISCHARGING);
  EXPECT_NEAR(soc0 - s.soc, s.current_a * 60.0 / (p.capacity_ah * 3600.0), 1.0e-12);
  EXPECT_LT(s.terminal_voltage_v, oru.open_circuit_voltage(soc0));
}

TEST(BatteryOru, ChargeAppliesCoulombicEfficiency)
{
  const BatteryOruParams p = default_battery_oru_params();
  BatteryOruModel oru(p);
  oru.reset(0.5, 10.0);
  const double absorbed = oru.step(60.0, -2000.0);
  const BatteryOruState & s = oru.state();
  EXPECT_NEAR(absorbed, -2000.0, 1.0e-6);
  EXPECT_LT(s.current_a, 0.0);
  EXPECT_EQ(s.mode, BatteryMode::CHARGING);
  EXPECT_NEAR(s.soc - 0.5,
              -s.current_a * p.coulombic_efficiency * 60.0 / (p.capacity_ah * 3600.0), 1.0e-12);
  EXPECT_GT(s.terminal_voltage_v, oru.open_circuit_voltage(0.5));
}

TEST(BatteryOru, CurrentLimitsAreRespected)
{
  const BatteryOruParams p = default_battery_oru_params();
  BatteryOruModel oru(p);
  oru.reset(0.5, 10.0);
  oru.step(1.0, 1.0e6);
  EXPECT_LE(oru.state().current_a, p.max_discharge_current_a + 1.0e-9);
  oru.step(1.0, -1.0e6);
  EXPECT_GE(oru.state().current_a, -p.max_charge_current_a - 1.0e-9);
}

TEST(BatteryOru, ConstantVoltageTaperNearFull)
{
  const BatteryOruParams p = default_battery_oru_params();
  BatteryOruModel oru(p);
  oru.reset(0.999, 10.0);
  oru.step(1.0, -1.0e6);
  const double v_max = p.max_cell_voltage_v * p.n_series_cells;
  EXPECT_LE(oru.state().terminal_voltage_v, v_max + 1.0e-6);
  EXPECT_LT(-oru.state().current_a, p.max_charge_current_a);
}

TEST(BatteryOru, SocStaysWithinBounds)
{
  BatteryOruModel oru;
  oru.reset(0.01, 10.0);
  for (int i = 0; i < 200; ++i) {
    oru.step(60.0, 5000.0);
    EXPECT_GE(oru.state().soc, 0.0);
  }
  EXPECT_DOUBLE_EQ(oru.max_discharge_power_w(60.0), 0.0);

  oru.reset(0.99, 10.0);
  for (int i = 0; i < 200; ++i) {
    oru.step(60.0, -5000.0);
    EXPECT_LE(oru.state().soc, 1.0);
  }
}

TEST(BatteryOru, FailedOruCarriesNoCurrent)
{
  BatteryOruModel oru;
  oru.set_failed(true);
  EXPECT_EQ(oru.state().health, BatteryHealth::FAILED);
  EXPECT_DOUBLE_EQ(oru.step(10.0, 3000.0), 0.0);
  EXPECT_DOUBLE_EQ(oru.state().current_a, 0.0);
  EXPECT_DOUBLE_EQ(oru.max_charge_power_w(10.0), 0.0);
  oru.set_failed(false);
  EXPECT_EQ(oru.state().health, BatteryHealth::GOOD);
  EXPECT_GT(oru.step(10.0, 3000.0), 0.0);
}

TEST(BatteryOru, ResistiveHeatingWarmsPackTowardEquilibrium)
{
  const BatteryOruParams p = default_battery_oru_params();
  BatteryOruModel oru(p);
  oru.reset(0.9, p.coldplate_temp_c);
  oru.step(600.0, 8000.0);
  const double heat = oru.state().heat_w;
  EXPECT_NEAR(heat, oru.state().current_a * oru.state().current_a *
              p.internal_resistance_ohm, 1.0e-9);
  EXPECT_GT(oru.state().temperature_c, p.coldplate_temp_c);
  EXPECT_LT(oru.state().temperature_c, p.coldplate_temp_c + heat / p.heat_transfer_w_k);
}

TEST(BatteryOru, OverTemperatureIsReported)
{
  BatteryOruParams p = default_battery_oru_params();
  p.coldplate_temp_c = 40.0;
  BatteryOruModel oru(p);
  oru.reset(0.5, 40.0);
  oru.step(1.0, 0.0);
  EXPECT_EQ(oru.state().health, BatteryHealth::OVERHEAT);
}

TEST(BatteryOru, StoredEnergyIsOcvIntegral)
{
  const BatteryOruParams p = default_battery_oru_params();
  BatteryOruModel oru(p);
  const int n = 100000;
  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    sum += oru.open_circuit_voltage((i + 0.5) / n) / n;
  }
  EXPECT_NEAR(oru.stored_energy_wh(1.0), sum * p.capacity_ah, 1.0e-3);
  EXPECT_DOUBLE_EQ(oru.stored_energy_wh(0.0), 0.0);
  // ~15 kWh for a 134 Ah, 30-cell pack.
  EXPECT_NEAR(oru.stored_energy_wh(1.0), 15.0e3, 1.0e3);
}

TEST(BatteryOru, DischargeEnergyIsConserved)
{
  const BatteryOruParams p = default_battery_oru_params();
  BatteryOruModel oru(p);
  const double e0 = oru.state().stored_energy_wh;
  double terminal_wh = 0.0;
  double heat_wh = 0.0;
  for (int i = 0; i < 1800; ++i) {
    terminal_wh += oru.step(1.0, 3000.0) / 3600.0;
    heat_wh += oru.state().heat_w / 3600.0;
  }
  const double drop = e0 - oru.state().stored_energy_wh;
  EXPECT_NEAR(drop, terminal_wh + heat_wh, 1.0e-3 * drop);
}
