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

EpsInput environment(double t)
{
  EpsInput in{};
  const double phase = std::fmod(t, kPeriodS) / kPeriodS;
  in.in_eclipse = phase > (1.0 - kEclipseFraction);
  in.solar_flux_w_m2 = in.in_eclipse ? 0.0 : 1361.0;
  return in;
}

EpsParameters zero_beta()
{
  EpsParameters p = default_eps_parameters();
  p.solar.array.use_fixed_beta = true;
  p.solar.array.fixed_beta_deg = 0.0;
  return p;
}

// Steps through the orbit from t until done() holds or max_s has passed.
template<typename Pred>
bool run_until(EpsSystem & eps, double & t, double max_s, Pred done)
{
  constexpr double dt = 10.0;
  for (const double end = t + max_s; t < end; t += dt) {
    if (done(eps.step(dt, environment(t)))) {
      return true;
    }
  }
  return false;
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
  const EpsState & s = eps.step(1.0, EpsInput{1361.0, false});
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
    s = eps.step(1.0, EpsInput{0.0, true});
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
    const EpsInput in = environment(t);
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
    const EpsState & s = eps.step(5.0, environment(t));
    double bus_discharge = 0.0;
    double bus_charge = 0.0;
    for (const auto & ch : s.channels) {
      bus_discharge += std::max(ch.bus_power_w, 0.0);
      bus_charge += std::max(-ch.bus_power_w, 0.0);
    }
    const double source = s.channels.front().source_available ? s.available_power_w : 0.0;
    EXPECT_NEAR(source + bus_discharge,
                s.primary_load_w + bus_charge + s.shunted_power_w, 1.0e-3);
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
    const EpsState & s = eps.step(1.0, EpsInput{0.0, true});
    load_wh += s.primary_load_w / 3600.0;
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
  p.demand_scale = 2.0;
  EpsSystem eps(p);
  bool saw_low_soc = false;
  bool saw_unserved = false;
  for (int i = 0; i < 4 * 3600; i += 10) {
    const EpsState & s = eps.step(10.0, EpsInput{0.0, true});
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
  eps.set_demand_scale(0.0);
  const double full = eps.step(1.0, EpsInput{1361.0, false}).generation_w;
  eps.set_array_degradation(0.5);
  const double half = eps.step(1.0, EpsInput{1361.0, false}).generation_w;
  EXPECT_NEAR(half, 0.5 * full, 1.0e-6);
}

TEST(EpsOrbit, GimbalLockOnlyByDrill)
{
  // Even at the peak of the beta cycle the gimbals keep tracking.
  EpsParameters p = zero_beta();
  p.solar.array.fixed_beta_deg = p.solar.array.beta_max_deg;
  EpsSystem eps(p);
  const double full = eps.step(1.0, EpsInput{1361.0, false}).generation_w;
  EXPECT_FALSE(eps.state().array.gimbal_locked);
  EXPECT_TRUE(eps.state().healthy);

  eps.set_gimbal_lock(true);
  const EpsState & locked = eps.step(1.0, EpsInput{1361.0, false});
  EXPECT_TRUE(locked.array.gimbal_locked);
  EXPECT_NEAR(locked.generation_w, p.solar.array.gimbal_lock_derate * full, 1.0e-6 * full);
  EXPECT_FALSE(locked.healthy);
  EXPECT_EQ(locked.status_message, "solar array gimbal locked");

  eps.set_gimbal_lock(false);
  EXPECT_TRUE(eps.step(1.0, EpsInput{1361.0, false}).healthy);
}

TEST(EpsOrbit, StartsSettledWithoutWarmUp)
{
  EpsSystem eps;
  const auto & p = eps.parameters();
  EXPECT_DOUBLE_EQ(eps.state().battery_soc, 0.95);

  const EpsState & s = eps.step(0.0, environment(0.0));
  for (const auto & d : s.ddcus) {
    EXPECT_NEAR(d.temperature_c, p.ddcu.coldplate_temp_c + d.heat_w / p.ddcu.heat_transfer_w_k,
                1.0);
  }
  for (const auto & o : s.orus) {
    EXPECT_NEAR(o.temperature_c,
                p.battery.coldplate_temp_c + o.heat_w / p.battery.heat_transfer_w_k, 1.0);
  }
  EXPECT_TRUE(s.healthy);

  // Constant load: the DDCUs hold their starting temperature.
  std::vector<double> start;
  for (const auto & d : s.ddcus) {
    start.push_back(d.temperature_c);
  }
  for (int i = 1; i <= 600; ++i) {
    eps.step(1.0, environment(i));
  }
  for (std::size_t d = 0; d < start.size(); ++d) {
    EXPECT_NEAR(eps.state().ddcus[d].temperature_c, start[d], 0.1) << "DDCU " << d;
  }
}

TEST(EpsOrbit, SixteenNominalOrbitsWithoutAFault)
{
  // Default beta precession (0 to ~8 deg over 16 orbits) and a fixed 5 deg.
  for (const bool fixed : {false, true}) {
    EpsParameters p = default_eps_parameters();
    p.solar.array.use_fixed_beta = fixed;
    p.solar.array.fixed_beta_deg = 5.0;
    EpsSystem eps(p);
    double max_ddcu_c = 0.0;
    double max_oru_c = 0.0;
    double min_soc = 1.0;
    int unhealthy_steps = 0;
    for (double t = 0.0; t < 16.0 * kPeriodS; t += 5.0) {
      const EpsState & s = eps.step(5.0, environment(t));
      unhealthy_steps += s.healthy ? 0 : 1;
      min_soc = std::min(min_soc, s.battery_soc);
      for (const auto & d : s.ddcus) {
        max_ddcu_c = std::max(max_ddcu_c, d.temperature_c);
      }
      for (const auto & o : s.orus) {
        max_oru_c = std::max(max_oru_c, o.temperature_c);
      }
    }
    EXPECT_EQ(unhealthy_steps, 0) << "fixed beta " << fixed;
    EXPECT_GT(min_soc, 0.9);
    EXPECT_LT(max_ddcu_c, p.ddcu.trip_temp_c - 10.0);
    EXPECT_LT(max_oru_c, p.battery.max_temp_c - 10.0);
    EXPECT_GE(eps.state().array.orbit_count, 15);
  }
}

TEST(EpsOrbit, OruOverheatDrillTripsAndRecovers)
{
  EpsSystem eps(zero_beta());
  double t = 0.0;
  eps.set_oru_extra_heat(4, 800.0);
  ASSERT_TRUE(run_until(eps, t, 2.0 * kPeriodS, [](const EpsState & s) {
      return s.oru_overheat;
    }));
  const EpsState & hot = eps.state();
  EXPECT_EQ(hot.orus[4].health, battery::BatteryHealth::OVERHEAT);
  EXPECT_EQ(hot.orus[5].health, battery::BatteryHealth::GOOD);
  EXPECT_FALSE(hot.healthy);
  EXPECT_FALSE(hot.load_unserved);
  EXPECT_EQ(hot.status_message, "battery ORU over temperature");

  eps.set_oru_extra_heat(4, 0.0);
  ASSERT_TRUE(run_until(eps, t, kPeriodS, [](const EpsState & s) {return s.healthy;}));
  EXPECT_LT(eps.state().orus[4].temperature_c, eps.parameters().battery.max_temp_c);
}

TEST(EpsOrbit, DdcuOverheatDrillTripsShedsAndRecovers)
{
  EpsSystem eps(zero_beta());
  double t = 0.0;
  eps.set_ddcu_extra_heat(0, 400.0);
  ASSERT_TRUE(run_until(eps, t, kPeriodS, [](const EpsState & s) {return s.ddcu_fault;}));
  const EpsState & hot = eps.state();
  EXPECT_EQ(hot.ddcus[0].mode, ddcu::DdcuMode::TRIPPED);
  EXPECT_TRUE(hot.ddcu_fault);
  EXPECT_TRUE(hot.load_shed);
  EXPECT_FALSE(hot.healthy);
  for (const auto & l : hot.loads) {
    EXPECT_EQ(l.shed, l.ddcu == 0) << l.id;
  }

  // The extra heat alone keeps it above the reset temperature.
  run_until(eps, t, 1800.0, [](const EpsState &) {return false;});
  EXPECT_TRUE(eps.state().ddcus[0].over_temperature);

  eps.set_ddcu_extra_heat(0, 0.0);
  ASSERT_TRUE(run_until(eps, t, kPeriodS, [](const EpsState & s) {return s.healthy;}));
  EXPECT_EQ(eps.state().ddcus[0].mode, ddcu::DdcuMode::REGULATING);
  EXPECT_LE(eps.state().ddcus[0].temperature_c, eps.parameters().ddcu.reset_temp_c);
}

TEST(EpsOrbit, BcduFaultIsReportedAndCleared)
{
  EpsSystem eps(zero_beta());
  eps.set_bcdu_fault(3, true);
  const EpsState & s = eps.step(1.0, EpsInput{0.0, true});
  EXPECT_TRUE(s.bcdu_fault);
  EXPECT_FALSE(s.healthy);
  EXPECT_EQ(s.channels[3].mode, bcdu::BcduMode::FAULT);
  // MBSU 1 reroutes channel 3's share onto channels 4 and 5.
  EXPECT_FALSE(s.load_unserved);
  EXPECT_DOUBLE_EQ(s.channels[3].load_served_w, 0.0);
  EXPECT_GT(s.channels[4].load_served_w, s.channels[0].load_served_w);

  eps.set_bcdu_fault(3, false);
  EXPECT_TRUE(eps.step(1.0, EpsInput{0.0, true}).healthy);
}

TEST(EpsOrbit, FailedOruIsReportedWithoutLoadShedding)
{
  EpsSystem eps(zero_beta());
  eps.set_oru_failed(5, true);
  const EpsState & s = eps.step(1.0, EpsInput{0.0, true});
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
    eps.step(1.0, EpsInput{0.0, true});
  }
  const double soc = eps.state().battery_soc;
  EpsParameters p = eps.parameters();
  p.bcdu.discharge_efficiency = 0.9;
  eps.set_parameters(p);
  EXPECT_NEAR(eps.state().battery_soc, soc, 1.0e-12);
  eps.step(1.0, EpsInput{0.0, true});
  EXPECT_NEAR(eps.state().battery_discharge_w, eps.state().primary_load_w / 0.9, 1.0);
}

TEST(EpsOrbit, OruOverTemperatureMakesEpsUnhealthy)
{
  EpsSystem eps(zero_beta());
  EXPECT_TRUE(eps.step(1.0, EpsInput{1361.0, false}).healthy);

  // Limit below the ORUs' current temperature.
  EpsParameters p = eps.parameters();
  p.battery.max_temp_c = eps.state().orus.front().temperature_c - 5.0;
  eps.set_parameters(p);
  const EpsState & hot = eps.step(1.0, EpsInput{1361.0, false});
  EXPECT_EQ(hot.orus.front().health, battery::BatteryHealth::OVERHEAT);
  EXPECT_TRUE(hot.oru_overheat);
  EXPECT_FALSE(hot.oru_failed);
  EXPECT_FALSE(hot.healthy);
  EXPECT_EQ(hot.status_message, "battery ORU over temperature");

  p.battery.max_temp_c = 35.0;
  eps.set_parameters(p);
  const EpsState & cool = eps.step(1.0, EpsInput{1361.0, false});
  EXPECT_FALSE(cool.oru_overheat);
  EXPECT_TRUE(cool.healthy);
}
