#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "ssos_eps/system/eps_system.hpp"

using namespace ssos_eps;
using namespace ssos_eps::system;

namespace
{
// Same eclipse geometry as the ssos_sim default world model.
constexpr double kPeriodS = 5546.0;
constexpr double kEclipseFraction = 0.35;

EpsInput environment(double t, double load_w)
{
  EpsInput in{};
  const double phase = std::fmod(t, kPeriodS) / kPeriodS;
  in.in_eclipse = phase > (1.0 - kEclipseFraction);
  in.solar_flux_w_m2 = in.in_eclipse ? 0.0 : 1361.0;
  in.load_demand_w = load_w;
  return in;
}

EpsParameters zero_beta()
{
  EpsParameters p = default_eps_parameters();
  p.solar.array.use_fixed_beta = true;
  p.solar.array.fixed_beta_deg = 0.0;
  return p;
}
}  // namespace

TEST(EpsOrbit, DefaultTopologyMatchesLegacyStation)
{
  EpsSystem eps;
  EXPECT_EQ(eps.n_channels(), 12);
  EXPECT_EQ(eps.n_orus(), 24);
  EXPECT_EQ(eps.state().orus.size(), 24u);
  EXPECT_EQ(eps.state().channels.size(), 12u);
}

TEST(EpsOrbit, SunlightCarriesLoadAndChargesBatteries)
{
  EpsSystem eps(zero_beta());
  const EpsState & s = eps.step(1.0, EpsInput{1361.0, false, 30000.0});
  EXPECT_GT(s.generation_w, 80000.0);
  EXPECT_NEAR(s.load_served_w, 30000.0, 1.0e-6);
  EXPECT_GT(s.battery_charge_w, 0.0);
  EXPECT_DOUBLE_EQ(s.battery_discharge_w, 0.0);
  EXPECT_GT(s.power_balance_w, 0.0);
  EXPECT_TRUE(s.healthy);
  for (const auto & ch : s.channels) {
    EXPECT_EQ(ch.mode, bcdu::BcduMode::CHARGING);
  }
}

TEST(EpsOrbit, EclipseDischargesBatteries)
{
  EpsSystem eps(zero_beta());
  const double soc0 = eps.state().battery_soc;
  EpsState s{};
  for (int i = 0; i < 600; ++i) {
    s = eps.step(1.0, EpsInput{0.0, true, 30000.0});
  }
  EXPECT_DOUBLE_EQ(s.generation_w, 0.0);
  EXPECT_NEAR(s.load_served_w, 30000.0, 1.0e-6);
  EXPECT_GT(s.battery_discharge_w, 30000.0);
  EXPECT_LT(s.battery_soc, soc0);
  EXPECT_LT(s.power_balance_w, 0.0);
  for (const auto & ch : s.channels) {
    EXPECT_EQ(ch.mode, bcdu::BcduMode::DISCHARGING);
    EXPECT_NEAR(ch.bus_voltage_v, 150.0, 1.0e-9);
  }
}

TEST(EpsOrbit, SocCyclesOverAnOrbitWithoutLoadShedding)
{
  EpsSystem eps(zero_beta());
  const double dt = 1.0;
  std::vector<double> soc_at_entry;
  std::vector<double> soc_at_exit;
  bool prev_eclipse = false;
  double unserved_wh = 0.0;
  for (double t = 0.0; t < 3.0 * kPeriodS; t += dt) {
    const EpsInput in = environment(t, 30000.0);
    const EpsState & s = eps.step(dt, in);
    if (in.in_eclipse && !prev_eclipse) {
      soc_at_entry.push_back(s.battery_soc);
    }
    if (!in.in_eclipse && prev_eclipse) {
      soc_at_exit.push_back(s.battery_soc);
    }
    unserved_wh += s.unserved_load_w * dt / 3600.0;
    prev_eclipse = in.in_eclipse;
  }
  ASSERT_GE(soc_at_entry.size(), 2u);
  ASSERT_GE(soc_at_exit.size(), 2u);
  // Each eclipse draws the bank down; the following daylight recharges it.
  EXPECT_LT(soc_at_exit[0], soc_at_entry[0]);
  EXPECT_LT(soc_at_exit[1], soc_at_entry[1]);
  EXPECT_GT(soc_at_entry[1], soc_at_exit[0]);
  EXPECT_LT(unserved_wh, 1.0e-6);
  EXPECT_TRUE(eps.state().healthy);
}

TEST(EpsOrbit, BusPowerBalancesEveryStep)
{
  EpsSystem eps;
  for (double t = 0.0; t < kPeriodS; t += 5.0) {
    const EpsState & s = eps.step(5.0, environment(t, 30000.0));
    double bus_discharge = 0.0;
    double bus_charge = 0.0;
    for (const auto & ch : s.channels) {
      bus_discharge += std::max(ch.bus_power_w, 0.0);
      bus_charge += std::max(-ch.bus_power_w, 0.0);
    }
    const double source = s.channels.front().source_available ? s.available_power_w : 0.0;
    EXPECT_NEAR(source + bus_discharge,
                s.load_served_w + bus_charge + s.shunted_power_w, 1.0e-3);
  }
}

TEST(EpsOrbit, EclipseEnergyIsConserved)
{
  EpsSystem eps(zero_beta());
  const double e0 = eps.state().stored_energy_wh;
  double load_wh = 0.0;
  double loss_wh = 0.0;
  double heat_wh = 0.0;
  for (int i = 0; i < 2000; ++i) {
    const EpsState & s = eps.step(1.0, EpsInput{0.0, true, 30000.0});
    load_wh += s.load_served_w / 3600.0;
    loss_wh += s.conversion_loss_w / 3600.0;
    for (const auto & o : s.orus) {
      heat_wh += o.heat_w / 3600.0;
    }
  }
  const double drop = e0 - eps.state().stored_energy_wh;
  EXPECT_NEAR(drop, load_wh + loss_wh + heat_wh, 1.0e-3 * drop);
}

TEST(EpsOrbit, OverloadDrainsBankAndDegrades)
{
  EpsParameters p = zero_beta();
  p.battery.initial_soc = 0.3;
  EpsSystem eps(p);
  bool saw_low_soc = false;
  bool saw_unserved = false;
  for (int i = 0; i < 4 * 3600; i += 10) {
    const EpsState & s = eps.step(10.0, EpsInput{0.0, true, 60000.0});
    saw_low_soc = saw_low_soc || s.low_soc;
    saw_unserved = saw_unserved || s.load_unserved;
  }
  EXPECT_TRUE(saw_low_soc);
  EXPECT_TRUE(saw_unserved);
  EXPECT_FALSE(eps.state().healthy);
  EXPECT_GE(eps.state().battery_soc, 0.0);
}

TEST(EpsOrbit, ArrayDegradationReducesGeneration)
{
  EpsSystem eps(zero_beta());
  const double full = eps.step(1.0, EpsInput{1361.0, false, 0.0}).generation_w;
  eps.set_array_degradation(0.5);
  const double half = eps.step(1.0, EpsInput{1361.0, false, 0.0}).generation_w;
  EXPECT_NEAR(half, 0.5 * full, 1.0e-6);
}

TEST(EpsOrbit, GimbalLockReducesGeneration)
{
  EpsParameters p = zero_beta();
  EpsSystem nominal(p);
  p.solar.array.fixed_beta_deg = 74.0;
  EpsSystem locked(p);
  const EpsState & a = nominal.step(1.0, EpsInput{1361.0, false, 30000.0});
  const EpsState & b = locked.step(1.0, EpsInput{1361.0, false, 30000.0});
  EXPECT_TRUE(b.array.gimbal_locked);
  EXPECT_LT(b.generation_w, 0.2 * a.generation_w);
}

TEST(EpsOrbit, BcduFaultIsReportedAndCleared)
{
  EpsSystem eps(zero_beta());
  eps.set_bcdu_fault(3, true);
  const EpsState & s = eps.step(1.0, EpsInput{0.0, true, 30000.0});
  EXPECT_TRUE(s.bcdu_fault);
  EXPECT_TRUE(s.load_unserved);
  EXPECT_FALSE(s.healthy);
  EXPECT_EQ(s.channels[3].mode, bcdu::BcduMode::FAULT);
  EXPECT_NEAR(s.unserved_load_w, 30000.0 / 12.0, 1.0e-6);

  eps.set_bcdu_fault(3, false);
  EXPECT_TRUE(eps.step(1.0, EpsInput{0.0, true, 30000.0}).healthy);
}

TEST(EpsOrbit, FailedOruIsReportedWithoutLoadShedding)
{
  EpsSystem eps(zero_beta());
  eps.set_oru_failed(5, true);
  const EpsState & s = eps.step(1.0, EpsInput{0.0, true, 30000.0});
  EXPECT_TRUE(s.oru_failed);
  EXPECT_FALSE(s.load_unserved);
  EXPECT_FALSE(s.healthy);
  EXPECT_DOUBLE_EQ(s.orus[5].current_a, 0.0);
  EXPECT_TRUE(s.channels[2].degraded);
}

TEST(EpsOrbit, LiveParameterUpdateKeepsBatteryState)
{
  EpsSystem eps(zero_beta());
  for (int i = 0; i < 600; ++i) {
    eps.step(1.0, EpsInput{0.0, true, 30000.0});
  }
  const double soc = eps.state().battery_soc;
  EpsParameters p = eps.parameters();
  p.bcdu.discharge_efficiency = 0.9;
  eps.set_parameters(p);
  EXPECT_NEAR(eps.state().battery_soc, soc, 1.0e-12);
  eps.step(1.0, EpsInput{0.0, true, 30000.0});
  EXPECT_NEAR(eps.state().battery_discharge_w, 30000.0 / 0.9, 1.0);
}
